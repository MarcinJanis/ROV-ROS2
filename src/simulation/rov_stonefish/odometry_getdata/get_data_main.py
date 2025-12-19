import sys
import time
import rclpy
from rclpy.executors import MultiThreadedExecutor
from threading import Thread

# Import Twoich modułów
# Upewnij się, że pliki ros_nodes.py i scenarios_reader.py są w tym samym folderze
from ros_nodes import StonefishPublisher, StonefishSubscriber
from scenarios_reader import MasterController

def spin_thread_func(executor):
    """Funkcja uruchamiana w osobnym wątku do obsługi callbacków ROS."""
    try:
        executor.spin()
    except Exception as e:
        print(f"[ROS Thread Info] Executor shutdown: {e}")

def main(args=None):
    # Wymuszenie natychmiastowego wypisywania printów (przydatne przy debugowaniu)
    sys.stdout.reconfigure(line_buffering=True)
    
    # 1. Inicjalizacja ROS2
    rclpy.init(args=args)
    print("[Main] ROS2 Initialized.")

    # 2. Tworzenie węzłów
    pub_node = StonefishPublisher()
    sub_node = StonefishSubscriber()

    # 3. Konfiguracja Executora (MultiThreaded dla lepszej wydajności)
    executor = MultiThreadedExecutor()
    executor.add_node(pub_node)
    executor.add_node(sub_node)

    # 4. Uruchomienie ROS w tle
    ros_thread = Thread(target=spin_thread_func, args=(executor,), daemon=True)
    ros_thread.start()
    print("[Main] Background ROS thread started.")

    # 5. Oczekiwanie na połączenie z symulatorem (Handshake)
    print("[Main] Waiting for connection with Stonefish simulator...")
    timeout = 10.0 # sekundy
    start_wait = time.time()
    connected = False

    while time.time() - start_wait < timeout:
        # Sprawdzamy, czy przyszły jakiekolwiek dane o pozycji
        if sub_node.POSITION is not None:
            connected = True
            print(f"[Main] Connected! Robot depth: {sub_node.POSITION[2]:.2f}m")
            break
        time.sleep(0.1) # Ważne: zwolnienie zasobów dla wątku ROS

    if not connected:
        print("[Error] Connection timeout! Check if simulator is running.")
        # Sprzątanie i wyjście
        executor.shutdown()
        pub_node.destroy_node()
        sub_node.destroy_node()
        rclpy.shutdown()
        return

    # 6. Konfiguracja Parametrów Eksperymentu
    # Granice sił i czasów dla losowych ruchów
    bounds = {
        't_min': 2.0,   't_max': 20.0,    # Czas trwania jednej akcji [s]
        'F_min': 40.0,  'F_max': 80.0,   # Siła liniowa [N] (Forward, Slide)
        'T_min': 0.1,   'T_max': 0.3,    # Moment obrotowy [Nm] (Yaw)
        'max_depth': 16.0                # Maksymalna głębokość [m]
    }

    # Katalog zapisu danych
    dataset_folder = "./dataset_output"

    # Inicjalizacja Kontrolera
    controller = MasterController(pub_node, sub_node, general_dir=dataset_folder)

    # Setup sekwencji
    # seq_id: numer sekwencji (folderu)
    # mv_count: ile różnych akcji ma wylosować w harmonogramie (dla trybu determinist=False)
    controller.setup(seq_id=1, determinist=False, mv_count=20, boundaries=bounds)

    print("[Main] Controller setup complete. Starting data collection...")
    print("-------------------------------------------------------------")

    # 7. Główna pętla wykonawcza
    try:
        # Zbieramy np. 1000 próbek (zdjęć + pomiarów)
        controller.sequence_exec(target_samples_num=1000)

    except KeyboardInterrupt:
        print("\n[Main] Interrupted by user (Ctrl+C).")
    
    except Exception as e:
        print(f"[Main] Unexpected Error: {e}")

    finally:
        # 8. Bezpieczne zamknięcie (Safety Shutdown)
        print("-------------------------------------------------------------")
        print("[Main] Stopping robot engines...")
        # Kilkukrotne wysłanie zer, żeby mieć pewność
        for _ in range(3):
            pub_node.send_cmd([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
            time.sleep(0.1)

        print("[Main] Shutting down ROS nodes...")
        try:
            executor.shutdown()
            pub_node.destroy_node()
            sub_node.destroy_node()
            rclpy.shutdown()
        except:
            pass
        print("[Main] Done.")

if __name__ == '__main__':
    main()
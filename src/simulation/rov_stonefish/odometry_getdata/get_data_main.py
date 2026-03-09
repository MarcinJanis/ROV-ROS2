import sys
import time
import rclpy
from rclpy.executors import MultiThreadedExecutor
from threading import Thread
from ros_nodes import StonefishPublisher, StonefishSubscriber
from scenarios_reader import MasterController

SEQ_ID = 3
target_samples_num = 50
scenario_pth = '/home/dev/ros2_ws/dataset/data/seq_1/control_sequence.xml' # uncomment for deterministic mode

def spin_thread_func(executor):
    """Funkcja uruchamiana w osobnym wątku do obsługi callbacków ROS."""
    try:
        executor.spin()
    except Exception as e:
        print(f"[ROS Thread Info] Executor shutdown: {e}")

def main(args=None):
    sys.stdout.reconfigure(line_buffering=True)
    
    rclpy.init(args=args)
    print("[Main] ROS2 Initialized.")

    pub_node = StonefishPublisher()
    sub_node = StonefishSubscriber()

    # MutiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(pub_node)
    executor.add_node(sub_node)

    ros_thread = Thread(target=spin_thread_func, args=(executor,), daemon=True)
    ros_thread.start()
    print("[Main] Background ROS thread started.")

    print("[Main] Waiting for connection with Stonefish simulator...")
    timeout = 10.0 # sekundy
    start_wait = time.time()
    connected = False

    while time.time() - start_wait < timeout:
        if sub_node.POSITION is not None:
            connected = True
            print(f"[Main] Connected! Robot depth: {sub_node.POSITION[2]:.2f}m")
            break
        time.sleep(0.1) 

    if not connected:
        print("[Error] Connection timeout! Check if simulator is running.")
        executor.shutdown()
        pub_node.destroy_node()
        sub_node.destroy_node()
        rclpy.shutdown()
        return


    bounds = {
        't_min': 10.0,   't_max': 20.0,      # Czas trwania jednej akcji [s]
        'F_min': 30.0,  'F_max': 80.0,      # Siła liniowa [N] (Forward, Slide)
        'T_min': 0.1,   'T_max': 0.2,       # Moment obrotowy [Nm] (Yaw)
        'max_depth': 15.0, 'min_depth': 10.0 # Maksymalna głębokość [m]
    }

    dataset_folder = "./dataset/data"

    controller = MasterController(pub_node, sub_node, general_dir=dataset_folder)

    # controller.setup(seq_id=SEQ_ID, determinist=False, mv_count=10, boundaries=bounds) # uncomment for undeterministic mode
    
    controller.setup(seq_id=SEQ_ID, determinist = True, scenario_pth = scenario_pth, mv_count=10, boundaries=bounds) # uncomment for deterministic mode 
    print("[Main] Controller setup complete. Starting data collection...")
    print("-------------------------------------------------------------")

    
    try:
        controller.sequence_exec(target_samples_num=target_samples_num)

    except KeyboardInterrupt:
        print("\n[Main] Interrupted by user (Ctrl+C).")
    
    except Exception as e:
        print(f"[Main] Unexpected Error: {e}")

    finally:
        print("-------------------------------------------------------------")
        print("[Main] Stopping robot engines...")
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
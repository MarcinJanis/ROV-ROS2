# from huggingface_hub import HfApi
# api = HfApi()

# # Twoje dane
# repo_id = "mjanis/SonarOdometry"
# token = "hf_sWEizmuDcwXhrPUDbRMdRoaoeEvEuTbpADE"
# folder_path = "/home/mjanis/workspace/ROV-ROS2/dataset" # Ścieżka do paczki, którą chcesz wysłać

# print(f"Rozpoczynam wysyłanie paczki z {folder_path}...")

# api.upload_folder(
#     folder_path=folder_path,
#     repo_id=repo_id,
#     repo_type="dataset",
#     path_in_repo="data_batch_1", # Tak nazwie się folder na Hugging Face
#     token=token
# )

# print("Gotowe! Paczka jest już na serwerze.")
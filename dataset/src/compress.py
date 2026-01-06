import subprocess
import os
import shutil

def get_size(path):
    """
    Returns the size of a file or a folder in bytes.
    """
    if os.path.isfile(path):
        # If it's a single file
        return os.path.getsize(path)
    elif os.path.isdir(path):
        # If it's a folder, sum all files inside
        total_size = 0
        for dirpath, dirnames, filenames in os.walk(path):
            for f in filenames:
                fp = os.path.join(dirpath, f)
                # Skip if it's a broken symbolic link
                if not os.path.islink(fp):
                    total_size += os.path.getsize(fp)
        return total_size
    return 0

# def compress(input_dir, output_file_no_ext):
#     """
#     Compresses a directory into a .tar.xz archive.
#     Note: shutil adds the extension automatically.
#     """
#     print(f"Starting compression of '{input_dir}'...")
#     try:
#         # 'xztar' uses LZMA2 compression (like 7-zip)
#         shutil.make_archive(output_file_no_ext, 'xztar', input_dir)
#         full_name = f"{output_file_no_ext}.tar.xz"
#         print(f"Compression finished successfully: {full_name}")
#         return full_name
#     except Exception as e:
#         print(f"Compression error: {e}")
#         return None

# def decompress(archive_path, extract_to):
#     """
#     Decompresses any supported archive (.tar.xz, .zip, .tar.gz) to a folder.
#     """
#     print(f"Starting decompression of '{archive_path}'...")
#     try:
#         # Create destination folder if it doesn't exist
#         os.makedirs(extract_to, exist_ok=True)
#         # Automatically detects the format (tar.xz, zip, etc.)
#         shutil.unpack_archive(archive_path, extract_to)
#         print(f"Decompression finished successfully to: {extract_to}")
#     except Exception as e:
#         print(f"Decompression error: {e}")
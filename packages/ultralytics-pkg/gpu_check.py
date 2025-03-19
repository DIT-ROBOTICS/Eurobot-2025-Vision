#!/usr/bin/env python3
import torch

is_cuda_available = torch.cuda.is_available()
print(f"Is CUDA available: {is_cuda_available}")
assert is_cuda_available, "CUDA is not available"

print(f"Current CUDA device: {torch.cuda.current_device()}")

print(f"Number of GPUs: {torch.cuda.device_count()}")

print(f"CUDA device name: {torch.cuda.get_device_name(torch.cuda.current_device())}")

print(f"CUDA memory allocated: {torch.cuda.memory_allocated() / (1024 ** 2):.2f} MB")
print(f"CUDA memory cached: {torch.cuda.memory_reserved() / (1024 ** 2):.2f} MB")

# print total GPU memory
if is_cuda_available:
    total_memory = torch.cuda.get_device_properties(torch.cuda.current_device()).total_memory
    print(f"Total GPU memory: {total_memory / (1024 ** 2):.2f} MB")
else:
    print("CUDA is not available.")
    
if torch.backends.cudnn.is_available():
    print("cuDNN is installed.")
else:
    print("cuDNN is not installed.")

print(torch.backends.cudnn.version())
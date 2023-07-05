#pragma once
#include <stdio.h>
#include "cuda_runtime_api.h"

#define cudaSafeCall(expr, return_code)                                     \
  {                                                                         \
    auto err = expr;                                                        \
    if (cudaSuccess != err) {                                               \
      printf("Error: %s\tfile(%s):%d\n", cudaGetErrorString(err), __FILE__, \
             __LINE__);                                                     \
      return int(return_code);                                              \
    }                                                                       \
  }
#define cudaSafeMalloc(gpu, size, ...)                    \
  if (cudaMalloc(&gpu, size) != cudaError::cudaSuccess) { \
    printf("%s cudaMalloc failed", #gpu);                 \
    return __VA_ARGS__;                                   \
  }

#define cudaSafeFree(gpu) if (gpu) cudaFree(gpu);

#define CUDACheck(func)                                                \
  {                                                                    \
    cudaError_t err = func;                                            \
    if (err != cudaSuccess) {                                          \
      printf("[%s:%d] CudaCheck Failed, error code (%s)!\n", __FILE__, \
             __LINE__, cudaGetErrorString(err));                       \
      exit(EXIT_FAILURE);                                              \
    }                                                                  \
  }

#define CUDAFreeWithLog(ptr)                                            \
  {                                                                     \
    cudaError_t err = cudaFree(ptr);                                    \
    if (err != cudaSuccess) {                                           \
      printf("[%s:%d]%s CudaFree Failed, error code (%s)!\n", __FILE__, \
             __LINE__, #ptr, cudaGetErrorString(err));                  \
      exit(EXIT_FAILURE);                                               \
    }                                                                   \
  }

namespace hesai {
inline void CudaInit() {
  // check the compute capability of the device
  int num_devices = 0;
  CUDACheck(cudaGetDeviceCount(&num_devices));
  printf("%d CUDA Device found\n", num_devices);
}

inline void CudaSyncWithCheck(const char* msg) {
  {
    cudaError_t err = cudaDeviceSynchronize();
    if (err != cudaSuccess) {
      printf("%s %s \n", msg, cudaGetErrorString(err));
    }
  }
}
}  // namespace hesai

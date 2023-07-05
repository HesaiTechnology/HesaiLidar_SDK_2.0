#include <memory>

#include "nvbuffer.h"
#include "cuda_runtime_api.h"

#define CUDACheck(func)                                                \
  {                                                                    \
    cudaError_t err = func;                                            \
    if (err != cudaSuccess) {                                          \
      printf("[%s:%d] CudaCheck Failed, error code (%s)!\n", __FILE__, \
             __LINE__, cudaGetErrorString(err));                       \
      exit(EXIT_FAILURE);                                              \
    }                                                                  \
  }
namespace hesai
{
namespace lidar
{
std::shared_ptr<MemBuffer> MemBuffer::New() {
#ifdef __QNX__
  std::shared_ptr<MemBuffer> ptr(new MemBufferQNX);
#else
  std::shared_ptr<MemBuffer> ptr(new MemBufferGPU);
#endif
  return ptr;
}

bool MemBufferGPU::OnInit() {
  if (cudaMalloc(&gpuPtr, size_) != cudaError::cudaSuccess) {
    printf("gpuPtr cudaMalloc failed L MemBufferGPU::OnInit\n");
    return false;
  }
  cpuPtr = malloc(size_);
  return true;
}

void* MemBufferGPU::CpuPtr() { return cpuPtr; }
void* MemBufferGPU::GpuPtr() { return gpuPtr; }
void MemBufferGPU::HostToDevice(int start, int size) {
  if (stream_) {
    CUDACheck(cudaMemcpyAsync((void*)((char*)gpuPtr + start),
                              (void*)((char*)cpuPtr + start), size,
                              cudaMemcpyHostToDevice, (cudaStream_t)stream_));
  } else {
    CUDACheck(cudaMemcpy((void*)((char*)gpuPtr + start),
                         (void*)((char*)cpuPtr + start), size,
                         cudaMemcpyHostToDevice));
  }
}
void MemBufferGPU::DeviceToHost(int start, int size) {
  if (stream_) {
    cudaMemcpyAsync((void*)((char*)cpuPtr + start),
                              (void*)((char*)gpuPtr + start), size,
                              cudaMemcpyDeviceToHost);
  } else {
    cudaMemcpy((void*)((char*)cpuPtr + start),
                         (void*)((char*)gpuPtr + start), size,
                         cudaMemcpyDeviceToHost);
  }
}

MemBufferGPU::~MemBufferGPU() {
  if (gpuPtr) cudaFree(gpuPtr);
  if (cpuPtr) free(cpuPtr);
}
}
}

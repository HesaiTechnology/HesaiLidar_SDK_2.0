/************************************************************************************************
Copyright (C) 2023 Hesai Technology Co., Ltd.
Copyright (C) 2023 Original Authors
All rights reserved.

All code in this repository is released under the terms of the following Modified BSD License. 
Redistribution and use in source and binary forms, with or without modification, are permitted 
provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this list of conditions and 
  the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice, this list of conditions and 
  the following disclaimer in the documentation and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its contributors may be used to endorse or 
  promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED 
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR 
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR 
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
************************************************************************************************/
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

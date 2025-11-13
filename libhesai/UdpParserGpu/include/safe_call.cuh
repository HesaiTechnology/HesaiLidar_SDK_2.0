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
#pragma once
#include <stdio.h>
#include "cuda_runtime_api.h"

#define cudaSafeCall(expr, return_code)                                     \
  {                                                                         \
    auto err = expr;                                                        \
    if (cudaSuccess != err) {                                               \
      LogError("Error: %s\t %d", cudaGetErrorString(err), \
             __LINE__);                                                     \
      return int(return_code);                                              \
    }                                                                       \
  }
  
inline void cudaSafeMalloc(void** gpu, size_t size) {        
  cudaError_t err = cudaMalloc(gpu, size);
  if (err != cudaSuccess) { 
    LogError("cudaMalloc failed, error code (%s)!", 
            cudaGetErrorString(err));                   
    gpu = nullptr;                                        
  }
}

#define cudaSafeFree(gpu) if (gpu) cudaFree(gpu);

inline int CUDACheck(cudaError_t err)                                                
{                                                                    
  if (err != cudaSuccess) {                                          
    LogError("CudaCheck Failed, error code (%s)!", 
            cudaGetErrorString(err));                       
  }                                                                  
  return int(err);                                                  
}

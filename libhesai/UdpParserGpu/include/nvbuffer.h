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
#include <stdint.h>
#include <iostream>
#include <memory>
namespace hesai
{
namespace lidar
{
class MemBuffer {
 protected:
  uint64_t size_;
  // typedef __device_builtin__ struct CUstream_st *cudaStream_t;
  // void* = cudaStream_t
  void* stream_;
  bool enable_cache_;

 public:
  static std::shared_ptr<MemBuffer> New();
  virtual ~MemBuffer() {}
  virtual void *GpuPtr() { return NULL; }
  virtual void *CpuPtr() { return NULL; }
  inline bool Init(uint64_t size, void *stream = NULL, bool enable_cache = true) {
    size_ = size;
    stream_ = stream;
    enable_cache_ = enable_cache;
    int res =  OnInit();
    return res;
  }
  virtual bool OnInit() { return true; }
  virtual void HostToDevice(int start, int size) {}
  virtual void DeviceToHost(int start, int size) {}
  virtual void HostToDevice() { HostToDevice(0, size_); }
  virtual void DeviceToHost() { DeviceToHost(0, size_); }
  // Wrap CPU || GPU operations between Wait & Signal
  // E.g.: 1.Wait 2.kernal_function 3.cudaDeviceSynchronize 4.Signal
  virtual void Signal() {}
  virtual void Wait() {}
  virtual uint64_t Size() const { return size_; }
  virtual void *stream() { return stream_; }
  virtual void flush(){ return; }
};
typedef std::shared_ptr<MemBuffer> MemBufferPtr;

class MemBufferSub : public MemBuffer {
 private:
  MemBufferPtr ptr_;
  int start_, sub_size_;

 public:
  MemBufferSub(const MemBufferSub &) = delete;
  void operator=(const MemBufferSub &) = delete;

 public:
  MemBufferSub(MemBufferPtr ptr, int start, int size)
      : ptr_(ptr), start_(start_), sub_size_(size) {}
  virtual void *GpuPtr() { return (char*)ptr_->GpuPtr() + start_; }
  virtual void *CpuPtr() { return (char*)ptr_->CpuPtr() + start_; }
  void HostToDevice(int start, int size) override {
    ptr_->HostToDevice(start, size);
  }
  void DeviceToHost(int start, int size) override {
    ptr_->DeviceToHost(start, size);
  }
  void HostToDevice() override {
    ptr_->HostToDevice(start_, sub_size_);
  }
  void DeviceToHost() override {
    ptr_->DeviceToHost(start_, sub_size_);
  }
  void Signal() override {
    ptr_->Signal();
  }
  void Wait() override {
    ptr_->Wait();
  }
  uint64_t Size() const override { return sub_size_; }
  void *stream() override { return ptr_->stream(); }
};

// Normal GPU + CPU memory
// user cudaMemcpy to transfer
class MemBufferGPU : public MemBuffer {
 private:
  void *cpuPtr;
  void *gpuPtr;
  MemBufferGPU(const MemBufferGPU &) = delete;
  void operator=(const MemBufferGPU &) = delete;

 public:
  MemBufferGPU() {}
  ~MemBufferGPU();
  bool OnInit() override;
  void *CpuPtr() override;
  void *GpuPtr() override;
  void HostToDevice(int start, int size) override;
  void DeviceToHost(int start, int size) override;
  void HostToDevice() override { HostToDevice(0, size_); }
  void DeviceToHost() override { DeviceToHost(0, size_); }
};

// QNX ZeroCopy Buffer
class MemBufferQNX : public MemBuffer {
 private:
  struct Private;
  Private *p_;
  MemBufferQNX(const MemBufferQNX &) = delete;
  void operator=(const MemBufferQNX &) = delete;

 public:
  MemBufferQNX();
  ~MemBufferQNX();
  bool OnInit() override;
  void *CpuPtr() override;
  void *GpuPtr() override;
  void Signal() override;
  void Wait() override;
  void flush() override;
};

template <typename T>
struct MemBufferArray {
  MemBufferPtr buf;
  MemBufferArray() {}
  MemBufferArray(uint64_t size, void *stream = NULL, bool force_gpu = false) {
    if (force_gpu) {
      buf.reset(new MemBufferGPU);
    } else {
      buf = MemBuffer::New();
    }
    if (!buf->Init(sizeof(T) * size, stream)) {
      printf("Failed to Init MemBufferArray\n ");
      abort();
    }
  }
  T *cpu() { return (T *)buf->CpuPtr(); }
  T *gpu() { return (T *)buf->GpuPtr(); }
  void Signal() { buf->Signal(); }
  void Wait() { buf->Wait(); }
  void HostToDevice() { buf->HostToDevice(); }
  void DeviceToHost() { buf->DeviceToHost(); }
  uint64_t Size() const { return buf->Size(); }
  void flush(){buf->flush();}
};

template <typename T>
struct MemBufferClass {
  MemBufferPtr buf;
  MemBufferClass(void *stream = NULL, bool force_gpu=false) {
    if (force_gpu) {
      buf.reset(new MemBufferGPU);
    } else {
      buf = MemBuffer::New();
    }
    if (!buf->Init(sizeof(T), stream)) {
      printf("Failed to Init MemBufferClass\n ");
      abort();
    }
  }
  T *cpu() { return (T *)buf->CpuPtr(); }
  T *gpu() { return (T *)buf->GpuPtr(); }
  void Signal() { buf->Signal(); }
  void Wait() { buf->Wait(); }
  void HostToDevice() { buf->HostToDevice(); }
  void DeviceToHost() { buf->DeviceToHost(); }
  uint64_t Size() const { return buf->Size(); }
  void flush(){buf->flush();}
};
}
}

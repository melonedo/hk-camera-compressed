#include "JPEGEncoder.hpp"
#include "bayer.hpp"
#include <cuda_runtime.h>
#include <nanobench.h>
#include <opencv2/opencv.hpp>
#include <stdlib.h>


cv::Mat get_image(const char *path) {
  auto image = cv::imread(path, cv::IMREAD_COLOR);
  image = ConvertBGR2BayerRG(image);
  // cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
  return image;
}

int encode_rgb(JPEGEncoder &encoder, const cv::Mat &image,
               bool verbose = false) {
  uint8_t *out;
  size_t len;
  encoder.param.perf_stats = verbose;
  double begin_time = gpujpeg_get_time();
  [[maybe_unused]] int rc = encoder.encode_rgb(image, &out, &len);
  assert(rc == GPUJPEG_NOERR);
  if (verbose) {
    encoder.print_stats(begin_time);
    printf("\n");
    FILE *outf = fopen("imgs/out.jpg", "wb");
    fwrite(out, len, 1, outf);
    fclose(outf);
    printf("Ouput imgs/out.jpg was written.\n");
  }
  return 0;
}

int encode_bayer(JPEGEncoder &encoder, const cv::Mat &image, bool async,
                 bool verbose = false) {
  uint8_t *out;
  size_t len;
  encoder.param.perf_stats = verbose;
  double begin_time = gpujpeg_get_time();
  [[maybe_unused]] int rc = encoder.encode_bayer(image, &out, &len, async);
  assert(rc == GPUJPEG_NOERR);
  if (verbose) {
    encoder.print_stats(begin_time);
    printf("\n");
    FILE *outf = fopen("imgs/out2.jpg", "wb");
    fwrite(out, len, 1, outf);
    fclose(outf);
    printf("Ouput imgs/out2.jpg was written.\n");
  }
  return 0;
}

int main() {
  // cv::Mat::setDefaultAllocator(cv::cuda::HostMem::getAllocator(
  //     cv::cuda::HostMem::AllocType::PAGE_LOCKED));
  // const char *path = "imgs/CS050-10UC.bmp";
  const char *path = "imgs/CS004-10UC.bmp";
  auto image = get_image(path);
  size_t total = image.elemSize() * image.total();
  printf("%s: %dx%dx%ld=%ld\n", path, image.cols, image.rows, image.elemSize(),
         total);
  void *buf;
  CHECK_CUDA(cudaMallocHost(&buf, total));
  // CHECK_CUDA(cudaHostAlloc(&buf, total, cudaHostAllocDefault));
  void *dev_buf;
  CHECK_CUDA(cudaMalloc(&dev_buf, total));
  JPEGEncoder encoder(0);
  auto b =
      ankerl::nanobench::Bench().minEpochTime(std::chrono::milliseconds(100));
  b.title("With cudaDeviceScheduleAuto");
  b.run("memcpy-cpu",
        [&] { memcpy(buf, image.data, image.elemSize() * image.total()); });
  b.run("memcpy-cuda", [&] {
    cudaMemcpy(buf, image.data, image.elemSize() * image.total(),
               cudaMemcpyHostToHost);
  });
  b.run("memcpy-pinned2dev", [&] {
    cudaMemcpy(dev_buf, buf, image.elemSize() * image.total(),
               cudaMemcpyHostToDevice);
  });
  b.run("memcpy-host2dev", [&] {
    cudaMemcpy(dev_buf, image.data, image.elemSize() * image.total(),
               cudaMemcpyHostToDevice);
  });
  cv::Mat image_rgb;
  cv::cvtColor(image, image_rgb, cv::COLOR_BayerRG2BGR); // ???
  b.run("encode-rgb-auto", [&] { encode_rgb(encoder, image_rgb); });
  b.run("encode-bayer-auto", [&] { encode_bayer(encoder, image, false); });
  b.run("encode-bayer-async-auto", [&] { encode_bayer(encoder, image, true); });
  cudaSetDeviceFlags(cudaDeviceScheduleBlockingSync);
  b.title("With cudaDeviceScheduleBlockingSync");
  b.run("encode-rgb", [&] { encode_rgb(encoder, image_rgb); });
  b.run("encode-bayer", [&] { encode_bayer(encoder, image, false); });
  b.run("encode-bayer-async",
        [&] { encode_bayer(encoder, image, true); });
  encode_rgb(encoder, image_rgb, true);
  encode_bayer(encoder, image, false, true);
  encode_bayer(encoder, image, true, true);
}

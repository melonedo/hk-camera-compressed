#include <cuda_runtime.h>

#include <atomic>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/opencv.hpp>

#include "libgpujpeg/gpujpeg.h"
#include "src/gpujpeg_encoder_internal.h"

#define CHECK_CUDA(call)                                                \
  {                                                                     \
    cudaError_t _e = (call);                                            \
    if (_e != cudaSuccess) {                                            \
      std::cout << "CUDA Runtime failure: '" << cudaGetErrorString(_e)  \
                << "' at " << __FILE__ << ":" << __LINE__ << std::endl; \
      exit(1);                                                          \
    }                                                                   \
  }

struct JPEGEncoder {
  struct gpujpeg_encoder *encoder;
  struct gpujpeg_parameters param;
  // struct gpujpeg_image_parameters param_image;

  JPEGEncoder(cudaStream_t s) {
    encoder = gpujpeg_encoder_create(s);
    assert(encoder);
    gpujpeg_set_default_parameters(&param);
    // Use 4:2:0 subsampling
    // gpujpeg_parameters_chroma_subsampling(&param, GPUJPEG_SUBSAMPLING_420);
    // param.verbose = true;
    param.restart_interval = RESTART_AUTO;
  }
  ~JPEGEncoder() {
    if (encoder) gpujpeg_encoder_destroy(encoder);
  }
  JPEGEncoder(const JPEGEncoder &) = delete;
  JPEGEncoder &operator=(const JPEGEncoder &) = delete;
  JPEGEncoder(JPEGEncoder &&other) noexcept
      : encoder(other.encoder), param(other.param) {
    other.encoder = nullptr;
  }
  JPEGEncoder &operator=(JPEGEncoder &&other) {
    if (encoder) gpujpeg_encoder_destroy(encoder);
    encoder = other.encoder;
    param = other.param;
    other.encoder = nullptr;
    return *this;
  }

  int encode(const struct gpujpeg_image_parameters *param_image,
             const struct gpujpeg_encoder_input *input_image,
             uint8_t **output_image, size_t *output_image_size) {
    return gpujpeg_encoder_encode(encoder, &param, param_image, input_image,
                                  output_image, output_image_size);
  }

  int encode_rgb(const cv::Mat &image, uint8_t **output_image,
                 size_t *output_image_size) {
    assert(image.elemSize() == 3 && image.isContinuous());
    struct gpujpeg_image_parameters param_image {
      image.cols, image.rows, GPUJPEG_RGB, GPUJPEG_444_U8_P012
    };
    struct gpujpeg_encoder_input encoder_input;
    gpujpeg_encoder_input_set_image(&encoder_input, image.data);
    return encode(&param_image, &encoder_input, output_image,
                  output_image_size);
  }

  int encode_bayer(const cv::Mat &image, uint8_t **output_image,
                   size_t *output_image_size, bool async = false) {
    cv::cuda::GpuMat gpu_image;
    gpu_image.upload(image);
    return encode_bayer(gpu_image, output_image, output_image_size, async);
  }

  // Convert Bayer RGGB8 to RGB and encode
  int encode_bayer(const cv::cuda::GpuMat &image, uint8_t **output_image,
                   size_t *output_image_size, bool async = false) {
    assert(image.elemSize() == 1);
    cv::cuda::GpuMat rgb;
    cv::cuda::demosaicing(image, rgb, cv::COLOR_BayerRG2RGB);
    // assert(rgb.isContinuous());
    void *buf;
    if (async) {
      CHECK_CUDA(
          cudaMallocAsync(&buf, image.cols * image.rows * 3, encoder->stream));
      CHECK_CUDA(cudaMemcpy2DAsync(buf, image.cols * 3, rgb.data, rgb.step,
                                   image.cols * 3, image.rows,
                                   cudaMemcpyDeviceToDevice, encoder->stream));
    } else {
      CHECK_CUDA(cudaMalloc(&buf, image.cols * image.rows * 3));
      CHECK_CUDA(cudaMemcpy2D(buf, image.cols * 3, rgb.data, rgb.step,
                              image.cols * 3, image.rows,
                              cudaMemcpyDeviceToDevice));
    }
    struct gpujpeg_image_parameters param_image {
      image.cols, image.rows, GPUJPEG_RGB, GPUJPEG_444_U8_P012
    };
    struct gpujpeg_encoder_input encoder_input;
    gpujpeg_encoder_input_set_gpu_image(&encoder_input, (uint8_t *)buf);
    int rc =
        encode(&param_image, &encoder_input, output_image, output_image_size);
    CHECK_CUDA(cudaFree(buf));
    return rc;
  }

  int print_stats(double begin_time) {
    double duration = gpujpeg_get_time() - begin_time;
    // duration_all_iterations += duration;
    // if (iteration == 0) {
    //   duration_first_iteration = duration;
    // }
    struct gpujpeg_duration_stats stats;
    int rc = gpujpeg_encoder_get_stats(encoder, &stats);
    if (rc != GPUJPEG_NOERR) {
      printf(" -Copy To Device:    %10.4f ms\n", stats.duration_memory_to);
      if (stats.duration_memory_map != 0.0 &&
          stats.duration_memory_unmap != 0.0) {
        printf(" -OpenGL Memory Map: %10.4f ms\n", stats.duration_memory_map);
        printf(" -OpenGL Memory Unmap:%9.4f ms\n", stats.duration_memory_unmap);
      }
      printf(" -Preprocessing:     %10.4f ms\n", stats.duration_preprocessor);
      printf(" -DCT & Quantization:%10.4f ms\n",
             stats.duration_dct_quantization);
      printf(" -Huffman Encoder:   %10.4f ms\n", stats.duration_huffman_coder);
      printf(" -Copy From Device:  %10.4f ms\n", stats.duration_memory_from);
      printf(" -Stream Formatter:  %10.4f ms\n", stats.duration_stream);

      printf("Encode Image GPU:    %10.4f ms (only in-GPU processing)\n",
             stats.duration_in_gpu);
      printf(
          "Encode Image Bare:   %10.4f ms (without copy to/from GPU memory)\n",
          duration * 1000.0 - stats.duration_memory_to -
              stats.duration_memory_from);
      printf("Encode Image:        %10.4f ms\n", duration * 1000.0);
    }
    return rc;
  }
};
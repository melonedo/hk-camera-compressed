#include "CameraMaster.hpp"
#include "MvCameraControl.h"
#include "hk-utils.hpp"
#include <cassert>
#include <csignal>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"

CameraMaster *pMaster;

template <typename F> int install_signal_handler(F f) {
  struct sigaction sa;
  sa.sa_handler = f;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;
  if (sigaction(SIGINT, &sa, nullptr) == -1) {
    perror("sigaction");
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}

int start_cameras() {
  int nRet = install_signal_handler([](int signal) {
    if (signal == SIGINT) {
      puts("SIGINT received. Exiting...");
      // Cleanup begin
      if (pMaster)
        pMaster->stop();
      // Cleanup end
      exit(EXIT_SUCCESS);
    }
  });

  if (EXIT_SUCCESS != nRet)
    return nRet;

  CameraMaster master;
  pMaster = &master;
  if (master.init(400))
    return EXIT_FAILURE;
  if (master.start_grabbing())
    return EXIT_FAILURE;

  fprintf(stderr, "Press enter or ctrl+c to exit.\n");
  while (getchar() != '\n')
    ;
  master.stop();
  pMaster = nullptr;
  return EXIT_SUCCESS;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  int nRet = start_cameras();

  rclcpp::shutdown();
  return nRet;
}

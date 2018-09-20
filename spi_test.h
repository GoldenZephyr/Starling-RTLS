



struct spi_bus {
  const char *interface_name;
  int spi_fd;
  struct spi_ioc_transfer *xfer;
};


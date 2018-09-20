



struct spi_bus {
  const char *interface_name;
  int spi_fd;
  struct spi_ioc_transfer *xfer;
};

int spi_init(struct spi_bus *bus);
int write_spi_msg(struct spi_bus *bus, char *rx, char *tx, int len);



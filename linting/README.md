Using uncrustify
================
e.g.
```
uncrustify -c ../../linting/uncrustify.cfg spi_comm_check.c spi_comm_check.c
```
Creates a .uncrustify file. Overwrite the original with it if you're happy.

To run oclint, do

```
<oclint_path> source/spi_utils.c source/single_message.c -- -c -Iinclude
```

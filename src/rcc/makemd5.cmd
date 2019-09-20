set md5file=version_bin_md5.inc
cat *.hpp | md5sum | cut -b-16 > %md5file%.tmp
<NUL set /p=const rcc_bin_version_t rcc_bin_version_id = 0x> %md5file%
cat %md5file%.tmp >> %md5file%
echo ; >> %md5file%
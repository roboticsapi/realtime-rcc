#!/bin/bash
md5file=version_bin_md5.inc
if [[ "$OSTYPE" == "linux-gnu" ]]; then
	md5s=`cat *.hpp | md5sum | cut -b-16`
elif [[ "$OSTYPE" == "darwin"* ]]; then
	md5s=`cat *.hpp | md5 | cut -b-16`
fi
echo const rcc_bin_version_t rcc_bin_version_id = 0x$md5s\; > $md5file

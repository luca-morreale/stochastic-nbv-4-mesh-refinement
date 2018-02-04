thesis

Tested on Ubuntu 16.04 LTS

Dependencies:
libproj-dev
libgsl-dev
libopengm-dev
libssh-dev

Troubleshooting:

error with libproj.so
sudo ln -s /usr/lib/libproj.so.0 /usr/lib/libproj.so

unable to find libvtkproj4.so
ln -s /usr/lib/x86_64-linux-gnu/libvtkCommonCore-6.2.so /usr/lib/libvtkproj4.so

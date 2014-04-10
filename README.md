magnet_arduino
==============

you'll want to update the submodules, linked to alex's github
git submodule update --init --recursive

this directory belongs in your [arduino folder]
libaries might need to be copied to [arduino folder]/libraries

specifically Alex used 
cp -R ./libraries/bglib/Arduino ../libraries/BGLib
"pwd" was /Users/alist/Documents/Arduino/magnet_arduino

you want to use arduino leonardo for the femptoduino ble113-enabled chip we're using from
http://www.femtoduino.com/spex/femtoduino-ble
doc from their site is avail in examples/fempto..



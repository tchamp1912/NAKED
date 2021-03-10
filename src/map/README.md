Steps to setup to generate a qr code:
* Open `gen_qr.cpp`
* Copy `simple()` function and change values to match desired environment
* Call this function from `main()`

Steps to gen a QR code:
* Nav to map directory (should be parent of this file)
* cmake .
* make
* ./gen_qr
* Qr code saved at qr.pgm
  * This can be converted to other file types easily using ImageMagick
    * brew install ImageMagick
    * `magick convert qr.pgm qr.png` etc.

1. 빌드 방법 
 $> make

 -> 빌드하면 'lgit_multi_rmnet.ko' 가 산출됩니다.

2. 드라이버 로드 방법
 $> modprobe usbnet
 $> insmod lgit_multi_rmnet.ko


3. 드라이버 언로드 방법
 $> rmmod lgit_multi_rmnet.ko
printf "%-10s\n"请输入相机通道
read id     # 选取相机通道
echo -e "It is switch to $id \n"        # -e 开启转义

printf "1.左单目模式：LEFT_EYE_MODE\n"
printf "2.右单目模式：RIGHT_EYE_MODE\n"
printf "3.红蓝模式：RED_BLUE_MODE\n"
printf "4.双目模式：BINOCULAR_MODE\n"

printf "%-10s\n"请输入相机输出模式序号
read mode   # 切换相机输出模式

printf "\n"

uvcdynctrl -d /dev/video${id} -S 6:8  '(LE)0x50ff'
uvcdynctrl -d /dev/video${id} -S 6:15 '(LE)0x00f6'
uvcdynctrl -d /dev/video${id} -S 6:8  '(LE)0x2500'
uvcdynctrl -d /dev/video${id} -S 6:8  '(LE)0x5ffe'
uvcdynctrl -d /dev/video${id} -S 6:15 '(LE)0x0003'
uvcdynctrl -d /dev/video${id} -S 6:15 '(LE)0x0002'
uvcdynctrl -d /dev/video${id} -S 6:15 '(LE)0x0012'
uvcdynctrl -d /dev/video${id} -S 6:15 '(LE)0x0004'
uvcdynctrl -d /dev/video${id} -S 6:8  '(LE)0x76c3'
uvcdynctrl -d /dev/video${id} -S 6:10 "(LE)0x0${mode}00"           # 切换相机输出模式

printf "\n"
echo -e "It is camera_mode $mode \n"
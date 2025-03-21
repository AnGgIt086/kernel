#!/bin/bash
compile_date=`date +%F`
compile_begin=`date +%s`
kernel_obj=''
solution=''
platform=''
project=''
touch Makefile
# touch cs_ndt/Makefile 
touch ../../mobileevent/drivers/Makefile
cd ../../../../../

echo "solution:${solution}, platform:${platform}, project:${project}"
#识别双屏项目
double_screen_project="PD2178 PD2229 PD2234 PD2234F_EX PD2266 PD2266F_EX" 
if [ `echo $double_screen_project | grep -c $project ` -gt 0 ]
then
	echo "double screen project!"
	export VTS_DOUBLE_SCREEN_PROJECT_DEBUG_COMPILE=yes
else
	echo "single screen project!"
fi

if [ $solution = "MTK" ] ; then
	platform=$(echo $platform | awk -F "full_" '{print $2}')
	kernel_obj="KERNEL_OBJ"

elif [ $solution = "QCOM" ] ; then
	kernel_obj="DLKM_OBJ"
fi

#编译前先删除 vivo_mb.ko cs_press.ko vivo_ts.ko
rm $vivo_mb_path
# rm $cs_press_path
rm $vivo_ts_path

if [ $solution = "MTK" ] ; then
	export BBK_PRODUCT_SOLUTION=MTK
elif [ $solution = "QCOM" ] ; then
	export BBK_PRODUCT_SOLUTION=QCOM
	dir_path=$(cd "$(dirname "$0")";pwd)
	echo ${dir_path}
fi

#Remove ko all debug sections
if test -s .llvm-strip_path;then
	echo ""
else
	find prebuilts/clang/host/linux-x86/ -name llvm-strip > .llvm-strip_path
fi
read -r line < .llvm-strip_path
$line --strip-debug $vivo_mb_path
$line --strip-debug $vivo_ts_path

if test -s $vivo_mb_path;then
echo $vivo_mb_path
fi
if test -s $vivo_ts_path;then
echo $vivo_ts_path
fi
# if test -s $cs_press_path;then
# echo $cs_press_path
# fi
echo ""

compile_end=`date +%s`
let compile_begin_to_end=($compile_end - $compile_begin)
echo "$compile_date, $compile_begin, $compile_end, $compile_begin_to_end, $project" >> ~/.CompileStatistics.csv



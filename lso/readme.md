type " ps -e | grep processname | awk '{printf "%s%s",temp,$1;temp=","}' | xargs terminator -x  top -p " to check the process on your screen !

linux sheel program is funny!

" ps -e | grep processname " will out the process pid 
" awk '{printf "%s%s",temp,$1;temp=","}' " will handel the pid whit a ','
" xargs terminator -x  top -p " will open a new window to show the top result

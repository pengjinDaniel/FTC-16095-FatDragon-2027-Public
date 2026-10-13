#!/bin/zsh
cd "${0:A:h}" || exit 1
/usr/bin/python3 ./upload-offline.py
task_exit=$?
print ""
read "?按回车关闭窗口……"
exit "$task_exit"

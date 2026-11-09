#!/bin/zsh
cd "${0:A:h}" || exit 1
if command -v python3 >/dev/null 2>&1; then
    python3 ./upload-offline.py "$@"
    task_exit=$?
else
    print '找不到 Python 3，请先安装 Python 3.8 或更新版本。'
    task_exit=127
fi
print ""
read "?按回车关闭窗口……"
exit "$task_exit"

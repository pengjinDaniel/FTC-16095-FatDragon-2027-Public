#!/bin/zsh
cd "${0:A:h}" || exit 1
main() {
    if ! command -v python3 >/dev/null 2>&1; then
        print '找不到 Python 3，请先安装 Python 3.8 或更新版本。'
        return 127
    fi
    if (( $# > 1 )); then
        print '用法：无线传程序.command [--check | --build-only]'
        return 2
    fi
    case "${1:-}" in
        --check) python3 ./upload-offline.py --check; return $? ;;
        ''|--build-only) ;;
        *) print '用法：无线传程序.command [--check | --build-only]'; return 2 ;;
    esac

    if [[ ! -x "${JAVA_HOME:-}/bin/java" ]]; then
        local java_candidate
        for java_candidate in "/Applications/Android Studio.app/Contents/jbr/Contents/Home" \
                "$HOME/Applications/Android Studio.app/Contents/jbr/Contents/Home"; do
            if [[ -x "$java_candidate/bin/java" ]]; then
                export JAVA_HOME="$java_candidate"
                break
            fi
        done
        if [[ ! -x "${JAVA_HOME:-}/bin/java" ]]; then
            export JAVA_HOME="$(/usr/libexec/java_home -v 17 2>/dev/null)"
        fi
    fi
    if [[ ! -x "${JAVA_HOME:-}/bin/java" ]]; then
        print '找不到可用 JDK，请安装 Android Studio 或设置 JAVA_HOME。未开始上传。'
        return 1
    fi
    # An unplugged external drive can leave the default Gradle home dangling.
    if [[ -z "${GRADLE_USER_HOME:-}" && -L "$HOME/.gradle" && ! -d "$HOME/.gradle" ]]; then
        export GRADLE_USER_HOME="${XDG_CACHE_HOME:-$HOME/.cache}/ftc1-gradle"
    fi
    print '先离线编译最新程序；成功后才进入上传。'
    ./gradlew clean :TeamCode:assembleDebug --offline --console=plain
    local build_exit=$?
    if (( build_exit != 0 )); then
        print '编译失败，已停止，不会上传旧 APK。若缺依赖，请先联网完成一次 Gradle 构建。'
        return "$build_exit"
    fi
    if [[ "${1:-}" == --build-only ]]; then
        python3 ./upload-offline.py --check
    else
        python3 ./upload-offline.py
    fi
}
main "$@"
task_exit=$?
print ""
if [[ -t 0 ]]; then
    read "?按回车关闭窗口……"
fi
exit "$task_exit"

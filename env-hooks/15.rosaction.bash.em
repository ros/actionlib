function _roscomplete_rosaction {
    local arg opts
    COMPREPLY=()
    arg="${COMP_WORDS[COMP_CWORD]}"

    if [[ $COMP_CWORD == 1 ]]; then
        opts="list type info send"
        COMPREPLY=($(compgen -W "$opts" -- ${arg}))
    elif [[ $COMP_CWORD == 2 ]]; then
        case ${COMP_WORDS[1]} in
            type|info|send)
                opts=$(rosaction list 2> /dev/null)
                COMPREPLY=($(compgen -W "$opts" -- ${arg}))
                ;;
        esac
    elif [[ $COMP_CWORD == 3 ]]; then
        case ${COMP_WORDS[1]} in
            send)
                type=$(rosaction type ${COMP_WORDS[2]})
                opts=$(rosmsg-proto msg 2> /dev/null -s ${type})
                if [ 0 -eq $? ]; then
                  COMPREPLY="$opts"
                fi
            ;;
        esac
    fi

}

complete -F "_roscomplete_rosaction" "rosaction"

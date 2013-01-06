#!/bin/sh

plain=0
internal=0
case "${1}" in
    "")         ;;
    --plain)    plain=1;;
    --internal) internal=1;;
    *)          printf "${0##*/}: unknow option '%s'\n" "${1}" >&2; exit 1;;
esac
if [ ${plain} -ne 0 -a ${internal} -ne 0 ]; then
    printf "Can't print both plain and internal" >&2
    printf " versions at the same time\n" >&2
    exit 1
fi

ver_file="${0%/*}/../.version"
k_ver="$(  head -n 1 "${ver_file}" |cut -d ' ' -f 1  )"
k_cset="$( head -n 1 "${ver_file}" |cut -d ' ' -f 2  )"
k_name="$( head -n 1 "${ver_file}" |cut -d ' ' -f 3- )"
kf_ver="$( tail -n 1 "${ver_file}"                   )"

if [ ${internal} -ne 0 ]; then
    printf "%s\n" "${kf_ver}"
    exit 0
fi

k_ver_plain="$( printf "%s" "${k_ver}"  \
                |sed -r -e 's/-rc.*//;' )"

case "${kf_ver}" in
    hg) kf_ver="hg_$( hg id -i -r . )"
        k_ver_extra="$( printf "_%-7.7s" "${k_cset}" )"
        ;;
    *)  k_ver_extra="";;
esac

if [ "${plain}" -eq 1 ]; then
    echo "${k_ver_plain}"
else
    echo "${k_ver}${k_ver_extra}.${kf_ver}"
fi


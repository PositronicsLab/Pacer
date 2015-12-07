#!/bin/bash
# replace all lines with form '1'['2']'3' with '2'
sed -i.bak 's#\(.*\)\[\(.*\)]\(.*\)#\2#' $1
# replace all lines with form '1'='2' with '2'
sed -i.bak 's#\(.*\)=\(.*\)#\2#' $1
# remove punctuation
sed -i.bak 's#,##g' $1
sed -i.bak 's#;##g' $1
#remove leading whitespace
sed -i.bak 's|^[[:blank:]]*||g' $1



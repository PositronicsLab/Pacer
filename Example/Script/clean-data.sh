grep "$1" $2 > $3
sed -i.bak "s# \]';##g" $3
sed -i.bak "s#.*= \[##g" $3

DIR="/media/stephany/Darryl_USyd/labeler/bin"
FILELIST=$(ls ${DIR})
n=0
for file in ${FILELIST}; do
    printf -v digit "%06d" $n
    printf "file %s \n" $file
    printf "new name %06d \n" $n
    mv "$DIR/${file}" "$DIR/${digit}.bin"
    n=$[n + 1]
done

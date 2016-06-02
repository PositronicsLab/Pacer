ffmpeg -i $1 -sws_flags lanczos+accurate_rnd -vf "scale=320:240" -c:v libx264 -crf 20 -preset veryslow -profile:v main -tune fastdecode -c:a copy $2

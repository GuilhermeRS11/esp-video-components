set(srcs "simple_video_server_example.c"
         "shell.c"
         "shell/camera_shell.c"
         "shell/simple_shell.c")
set(html_files "../frontend/gzipped/index.html.gz"
               "../frontend/gzipped/loading.jpg.gz"
               "../frontend/gzipped/favicon.ico.gz"
               "../frontend/gzipped/assets/index.js.gz"
               "../frontend/gzipped/assets/index.css.gz")

idf_component_register(SRCS "${srcs}"
                       PRIV_INCLUDE_DIRS . shell
                       EMBED_TXTFILES ${html_files})

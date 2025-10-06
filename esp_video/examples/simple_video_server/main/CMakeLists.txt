set(srcs
         "shell.c"
         "shell/camera_control_api.c"
         "shell/camera_shell_min.c"
         "shell/simple_shell.c")
set(html_files "../frontend/gzipped/index.html.gz"
               "../frontend/gzipped/loading.jpg.gz"
               "../frontend/gzipped/favicon.ico.gz"
               "../frontend/gzipped/assets/index.js.gz"
               "../frontend/gzipped/assets/index.css.gz")

# No IMX708 hardware shell when using minimal API-driven shell

idf_component_register(SRCS "simple_video_server_example.c" "${srcs}"
                       PRIV_INCLUDE_DIRS . shell
                       EMBED_TXTFILES ${html_files})

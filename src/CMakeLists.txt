SET(SOURCES screens/ui_Screen1.c
    screens/ui_Screen3.c
    screens/ui_Screen2.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    images/ui_img_237497367.c
    images/ui_img_8884449.c
    images/ui_img_1662018118.c
    images/ui_img_up_png.c
    images/ui_img_down_png.c
    images/ui_img_bolt1_png.c
    images/ui_img_water1_png.c
    images/ui_img_1242781495.c)

add_library(ui ${SOURCES})

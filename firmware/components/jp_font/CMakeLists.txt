# Japanese-capable UI font (Noto Sans CJK JP, subset) for the speech bubble / UI.
#
# Generated from hiz8 "Noto-Sans-CJK-JP.min" 3.1.0 (Regular) with:
#   npx lv_font_conv --font NotoSansCJKjp-Regular.min.ttf --size 20 --bpp 4 \
#       --no-compress --format lvgl -o font_noto_jp_20_4.c --range 0x20-0xFFFF
# (--no-compress is required: CONFIG_LV_USE_FONT_COMPRESSED is disabled.)
#
# Selected via BUILTIN_TEXT_FONT=font_noto_jp_20_4 in main/CMakeLists.txt. The
# non-"basic" name also means build_default_assets.py ships no text_font .bin, so
# this compiled font is the one actually used (the assets font does not override it).
idf_component_register(
    SRCS "font_noto_jp_20_4.c"
    PRIV_REQUIRES "lvgl"
)

# The generated .c selects #include "lvgl.h" only under this guard.
target_compile_definitions(${COMPONENT_LIB} PRIVATE LV_LVGL_H_INCLUDE_SIMPLE)

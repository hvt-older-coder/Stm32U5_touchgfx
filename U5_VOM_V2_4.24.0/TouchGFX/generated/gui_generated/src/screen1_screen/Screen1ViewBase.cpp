/*********************************************************************************/
/********** THIS FILE IS GENERATED BY TOUCHGFX DESIGNER, DO NOT MODIFY ***********/
/*********************************************************************************/
#include <gui_generated/screen1_screen/Screen1ViewBase.hpp>
#include <touchgfx/Color.hpp>
#include <images/BitmapDatabase.hpp>
#include <texts/TextKeysAndLanguages.hpp>

Screen1ViewBase::Screen1ViewBase()
{
    __background.setPosition(0, 0, 240, 320);
    __background.setColor(touchgfx::Color::getColorFromRGB(0, 0, 0));
    add(__background);

    box1.setPosition(0, 0, 240, 320);
    box1.setColor(touchgfx::Color::getColorFromRGB(0, 0, 0));
    add(box1);

    box2.setPosition(0, 7, 240, 71);
    box2.setColor(touchgfx::Color::getColorFromRGB(0, 0, 0));
    add(box2);

    gauge1.setBackground(touchgfx::Bitmap(BITMAP_ALTERNATE_THEME_IMAGES_WIDGETS_GAUGE_MEDIUM_BACKGROUNDS_LIGHT_FILLED_ID));
    gauge1.setPosition(0, 80, 240, 240);
    gauge1.setCenter(120, 120);
    gauge1.setStartEndAngle(-113, 112);
    gauge1.setRange(0, 100);
    gauge1.setValue(50);
    gauge1.setNeedle(BITMAP_ALTERNATE_THEME_IMAGES_WIDGETS_GAUGE_MEDIUM_NEEDLES_SMOOTH_ID, 7, 67);
    gauge1.setMovingNeedleRenderingAlgorithm(touchgfx::TextureMapper::BILINEAR_INTERPOLATION);
    gauge1.setSteadyNeedleRenderingAlgorithm(touchgfx::TextureMapper::BILINEAR_INTERPOLATION);
    add(gauge1);

    textArea1.setXY(3, 21);
    textArea1.setColor(touchgfx::Color::getColorFromRGB(13, 101, 243));
    textArea1.setLinespacing(0);
    Unicode::snprintf(textArea1Buffer, TEXTAREA1_SIZE, "%s", touchgfx::TypedText(T___SINGLEUSE_HDBH).getText());
    textArea1.setWildcard(textArea1Buffer);
    textArea1.resizeToCurrentText();
    textArea1.setTypedText(touchgfx::TypedText(T_VOLTAGETEXTID));
    add(textArea1);

    textArea2.setXY(5, -2);
    textArea2.setColor(touchgfx::Color::getColorFromRGB(13, 101, 243));
    textArea2.setLinespacing(0);
    Unicode::snprintf(textArea2Buffer, TEXTAREA2_SIZE, "%s", touchgfx::TypedText(T___SINGLEUSE_XJHD).getText());
    textArea2.setWildcard(textArea2Buffer);
    textArea2.resizeToCurrentText();
    textArea2.setTypedText(touchgfx::TypedText(T_CURRENTTEXTID));
    add(textArea2);

    textArea2_1.setXY(161, 45);
    textArea2_1.setColor(touchgfx::Color::getColorFromRGB(255, 0, 0));
    textArea2_1.setLinespacing(0);
    Unicode::snprintf(textArea2_1Buffer, TEXTAREA2_1_SIZE, "%s", touchgfx::TypedText(T___SINGLEUSE_7DEW).getText());
    textArea2_1.setWildcard(textArea2_1Buffer);
    textArea2_1.resizeToCurrentText();
    textArea2_1.setTypedText(touchgfx::TypedText(T_FPSTEXTID));
    add(textArea2_1);

    textArea3.setXY(3, 45);
    textArea3.setColor(touchgfx::Color::getColorFromRGB(13, 101, 243));
    textArea3.setLinespacing(0);
    Unicode::snprintf(textArea3Buffer, TEXTAREA3_SIZE, "%s", touchgfx::TypedText(T___SINGLEUSE_7NX2).getText());
    textArea3.setWildcard(textArea3Buffer);
    textArea3.resizeToCurrentText();
    textArea3.setTypedText(touchgfx::TypedText(T_POWERTEXTID));
    add(textArea3);
}

Screen1ViewBase::~Screen1ViewBase()
{

}

void Screen1ViewBase::setupScreen()
{

}

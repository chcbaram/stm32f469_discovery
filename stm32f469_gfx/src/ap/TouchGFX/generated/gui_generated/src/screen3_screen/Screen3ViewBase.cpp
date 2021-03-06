/*********************************************************************************/
/********** THIS FILE IS GENERATED BY TOUCHGFX DESIGNER, DO NOT MODIFY ***********/
/*********************************************************************************/
#include <gui_generated/screen3_screen/Screen3ViewBase.hpp>
#include <touchgfx/Color.hpp>
#include "BitmapDatabase.hpp"
#include <texts/TextKeysAndLanguages.hpp>

Screen3ViewBase::Screen3ViewBase() :
    buttonCallback(this, &Screen3ViewBase::buttonCallbackHandler)
{
    box1.setPosition(0, 0, 800, 480);
    box1.setColor(touchgfx::Color::getColorFrom24BitRGB(255, 255, 255));

    buttonWithIcon1.setXY(630, 0);
    buttonWithIcon1.setBitmaps(Bitmap(BITMAP_BLUE_BUTTONS_ROUND_EDGE_SMALL_ID), Bitmap(BITMAP_BLUE_BUTTONS_ROUND_EDGE_SMALL_PRESSED_ID), Bitmap(BITMAP_BLUE_ICONS_HOME_32_ID), Bitmap(BITMAP_BLUE_ICONS_HOME_32_ID));
    buttonWithIcon1.setIconXY(70, 16);
    buttonWithIcon1.setAction(buttonCallback);

    textArea1.setXY(372, 228);
    textArea1.setColor(touchgfx::Color::getColorFrom24BitRGB(0, 0, 0));
    textArea1.setLinespacing(0);
    textArea1.setTypedText(TypedText(T_SINGLEUSEID4));

    add(box1);
    add(buttonWithIcon1);
    add(textArea1);
}

void Screen3ViewBase::setupScreen()
{

}

void Screen3ViewBase::buttonCallbackHandler(const touchgfx::AbstractButton& src)
{
    if (&src == &buttonWithIcon1)
    {
        //Interaction1
        //When buttonWithIcon1 clicked change screen to Screen1
        //Go to Screen1 with screen transition towards West
        application().gotoScreen1ScreenSlideTransitionWest();
    }
}

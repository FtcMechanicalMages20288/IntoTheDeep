package com.example.meepmeeptesting.core.colorscheme.scheme

import com.noahbres.meepmeep.core.colorscheme.ColorManager
import com.noahbres.meepmeep.core.colorscheme.ColorScheme
import java.awt.Color

open class ColorSchemeRedLight : ColorScheme() {
    override val isDark: Boolean = false

    override val BOT_BODY_COLOR = ColorManager.COLOR_PALETTE.RED_600
    override val BOT_WHEEL_COLOR = ColorManager.COLOR_PALETTE.RED_900
    override val BOT_DIRECTION_COLOR = ColorManager.COLOR_PALETTE.RED_900

    override val AXIS_X_COLOR: Color = ColorManager.COLOR_PALETTE.GRAY_900
    override val AXIS_Y_COLOR: Color = ColorManager.COLOR_PALETTE.GRAY_900

    override val TRAJECTORY_PATH_COLOR: Color = ColorManager.COLOR_PALETTE.BLUE_500
    override val TRAJECTORY_TURN_COLOR: Color = ColorManager.COLOR_PALETTE.PINK_600
    override val TRAJECTORY_MARKER_COLOR: Color = ColorManager.COLOR_PALETTE.ORANGE_600

    override val AXIS_NORMAL_OPACITY: Double = 0.3
    override val AXIS_HOVER_OPACITY: Double = 0.8

    override val TRAJECTORY_SLIDER_BG: Color = ColorManager.COLOR_PALETTE.GRAY_100
    override val TRAJECTORY_SLIDER_FG: Color = ColorManager.COLOR_PALETTE.RED_600
    override val TRAJECTORY_TEXT_COLOR: Color = ColorManager.COLOR_PALETTE.GRAY_900

    override val UI_MAIN_BG: Color = ColorManager.COLOR_PALETTE.GRAY_100
}
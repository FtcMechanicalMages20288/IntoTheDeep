package com.example.meepmeeptesting.core.entity

import com.noahbres.meepmeep.core.colorscheme.ColorScheme

interface ThemedEntity: Entity {
    fun switchScheme(scheme: ColorScheme)
}

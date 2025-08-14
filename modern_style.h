#pragma once
#include <QString>

namespace MaterialStyle {
    
const QString getStyleSheet() {
    return R"(
/* ===== MATERIAL DESIGN UI STYLE ===== */

/* Material Design Color Palette */
/* Primary: #1976D2 (Blue), Secondary: #DC004E (Pink), Surface: #FFFFFF, Background: #FAFAFA */

/* Main Window Background */
QMainWindow {
    background-color: #FAFAFA;
    color: #212121;
    font-family: 'Roboto', 'Segoe UI', 'Arial', sans-serif;
    font-size: 14px;
    font-weight: 400;
}

QWidget {
    background-color: #FAFAFA;
    color: #212121;
    font-family: 'Roboto', 'Segoe UI', 'Arial', sans-serif;
}

/* ===== MATERIAL BUTTONS ===== */
QPushButton {
    background-color: #FFFFFF;
    border: none;
    border-radius: 4px;
    color: #1976D2;
    font-weight: 500;
    font-size: 14px;
    text-transform: uppercase;
    letter-spacing: 0.75px;
    padding: 12px 24px;
    min-height: 36px;
    min-width: 120px;
    /* Material elevation 2 */
    box-shadow: 0px 3px 1px -2px rgba(0,0,0,0.2), 
                0px 2px 2px 0px rgba(0,0,0,0.14), 
                0px 1px 5px 0px rgba(0,0,0,0.12);
}

QPushButton:hover {
    /* Material elevation 4 */
    box-shadow: 0px 2px 4px -1px rgba(0,0,0,0.2), 
                0px 4px 5px 0px rgba(0,0,0,0.14), 
                0px 1px 10px 0px rgba(0,0,0,0.12);
    background-color: rgba(25, 118, 210, 0.04);
}

QPushButton:pressed {
    /* Material elevation 8 */
    box-shadow: 0px 5px 5px -3px rgba(0,0,0,0.2), 
                0px 8px 10px 1px rgba(0,0,0,0.14), 
                0px 3px 14px 2px rgba(0,0,0,0.12);
    background-color: rgba(25, 118, 210, 0.12);
}

QPushButton:disabled {
    background-color: rgba(0,0,0,0.04);
    color: rgba(0,0,0,0.26);
    box-shadow: none;
}

/* Material Contained Buttons (Start Actions) */
QPushButton[objectName*="start"], QPushButton[objectName*="Start"] {
    background-color: #1976D2;
    color: #FFFFFF;
}

QPushButton[objectName*="start"]:hover, QPushButton[objectName*="Start"]:hover {
    background-color: #1565C0;
}

QPushButton[objectName*="start"]:pressed, QPushButton[objectName*="Start"]:pressed {
    background-color: #0D47A1;
}

/* Material Error Buttons (Stop Actions) */
QPushButton[objectName*="stop"], QPushButton[objectName*="Stop"] {
    background-color: #D32F2F;
    color: #FFFFFF;
}

QPushButton[objectName*="stop"]:hover, QPushButton[objectName*="Stop"]:hover {
    background-color: #C62828;
}

QPushButton[objectName*="stop"]:pressed, QPushButton[objectName*="Stop"]:pressed {
    background-color: #B71C1C;
}

/* Material Secondary Buttons (Recording) */
QPushButton[objectName*="Recording"] {
    background-color: #DC004E;
    color: #FFFFFF;
}

QPushButton[objectName*="Recording"]:hover {
    background-color: #C2185B;
}

QPushButton[objectName*="Recording"]:pressed {
    background-color: #AD1457;
}

/* ===== MATERIAL CARDS/SURFACES ===== */
QLabel[objectName*="Status"] {
    background-color: #FFFFFF;
    border: none;
    border-radius: 8px;
    color: #212121;
    font-weight: 500;
    font-size: 14px;
    padding: 16px;
    min-height: 24px;
    /* Material elevation 1 */
    box-shadow: 0px 2px 1px -1px rgba(0,0,0,0.2), 
                0px 1px 1px 0px rgba(0,0,0,0.14), 
                0px 1px 3px 0px rgba(0,0,0,0.12);
}

/* ===== MATERIAL TYPOGRAPHY ===== */
QLabel {
    color: #212121;
    font-size: 14px;
    font-weight: 400;
    line-height: 1.43;
}

/* Material Headline 6 for titles */
QLabel[objectName*="title"], QLabel[objectName*="Title"] {
    font-size: 20px;
    font-weight: 500;
    line-height: 1.6;
    letter-spacing: 0.15px;
}

/* Material Body 2 for emphasis */
QLabel[objectName*="Status"] {
    font-size: 14px;
    font-weight: 500;
    line-height: 1.43;
    letter-spacing: 0.25px;
}

/* ===== MATERIAL CHECKBOXES ===== */
QCheckBox {
    color: #212121;
    font-size: 14px;
    font-weight: 400;
    spacing: 12px;
}

QCheckBox::indicator {
    width: 20px;
    height: 20px;
    border-radius: 2px;
    border: 2px solid #757575;
    background-color: transparent;
}

QCheckBox::indicator:checked {
    background-color: #1976D2;
    border-color: #1976D2;
    image: url(data:image/svg+xml;base64,PHN2ZyB3aWR0aD0iMTIiIGhlaWdodD0iMTIiIHZpZXdCb3g9IjAgMCAxMiAxMiIgZmlsbD0ibm9uZSIgeG1sbnM9Imh0dHA6Ly93d3cudzMub3JnLzIwMDAvc3ZnIj4KPHBhdGggZD0iTTEwIDNMNC41IDguNUwyIDYiIHN0cm9rZT0id2hpdGUiIHN0cm9rZS13aWR0aD0iMiIgc3Ryb2tlLWxpbmVjYXA9InJvdW5kIiBzdHJva2UtbGluZWpvaW49InJvdW5kIi8+Cjwvc3ZnPgo=);
}

QCheckBox::indicator:hover {
    background-color: rgba(25, 118, 210, 0.04);
}

QCheckBox::indicator:checked:hover {
    background-color: #1565C0;
}

/* ===== MATERIAL CONTAINERS ===== */
QStackedWidget {
    background-color: #FFFFFF;
    border: none;
    border-radius: 8px;
    /* Material elevation 1 */
    box-shadow: 0px 2px 1px -1px rgba(0,0,0,0.2), 
                0px 1px 1px 0px rgba(0,0,0,0.14), 
                0px 1px 3px 0px rgba(0,0,0,0.12);
}

QStackedWidget > QWidget {
    background-color: #FFFFFF;
    border-radius: 8px;
    padding: 16px;
}

QFrame {
    background-color: transparent;
    border: none;
}

/* ===== MATERIAL DIVIDERS ===== */
QFrame[frameShape="4"] { /* HLine */
    background-color: rgba(0,0,0,0.12);
    max-height: 1px;
    border: none;
}

QFrame[frameShape="5"] { /* VLine */
    background-color: rgba(0,0,0,0.12);
    max-width: 1px;
    border: none;
}

/* ===== MATERIAL SCROLLBARS ===== */
QScrollBar:vertical {
    background-color: transparent;
    width: 8px;
    border: none;
    border-radius: 4px;
}

QScrollBar::handle:vertical {
    background-color: rgba(0,0,0,0.3);
    border-radius: 4px;
    min-height: 20px;
}

QScrollBar::handle:vertical:hover {
    background-color: rgba(0,0,0,0.5);
}

QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
    height: 0px;
}

/* ===== MATERIAL SPECIFIC COLORS ===== */
/* Success State */
.success {
    background-color: #4CAF50 !important;
    color: #FFFFFF !important;
}

/* Warning State */
.warning {
    background-color: #FF9800 !important;
    color: #FFFFFF !important;
}

/* Error State */
.error {
    background-color: #F44336 !important;
    color: #FFFFFF !important;
}

/* Info State */
.info {
    background-color: #2196F3 !important;
    color: #FFFFFF !important;
}

/* ===== RVIZ CONTAINER ===== */
QWidget[objectName="rvizContainer"] {
    background-color: #FFFFFF;
    border: none;
    border-radius: 8px;
    /* Material elevation 2 */
    box-shadow: 0px 3px 1px -2px rgba(0,0,0,0.2), 
                0px 2px 2px 0px rgba(0,0,0,0.14), 
                0px 1px 5px 0px rgba(0,0,0,0.12);
}

/* ===== MATERIAL NAVIGATION ===== */
QPushButton[objectName*="Page"] {
    background-color: transparent;
    color: #1976D2;
    box-shadow: none;
    border-radius: 20px;
    min-width: 80px;
    padding: 8px 16px;
}

QPushButton[objectName*="Page"]:hover {
    background-color: rgba(25, 118, 210, 0.04);
}

QPushButton[objectName*="Page"]:pressed {
    background-color: rgba(25, 118, 210, 0.12);
}

/* ===== MATERIAL TOOLTIPS ===== */
QToolTip {
    background-color: #616161;
    color: #FFFFFF;
    border: none;
    border-radius: 4px;
    padding: 8px 12px;
    font-size: 12px;
    font-weight: 500;
    /* Material elevation 6 */
    box-shadow: 0px 3px 5px -1px rgba(0,0,0,0.2), 
                0px 6px 10px 0px rgba(0,0,0,0.14), 
                0px 1px 18px 0px rgba(0,0,0,0.12);
}

/* ===== MATERIAL FOCUS STATES ===== */
QPushButton:focus, QCheckBox:focus {
    outline: none;
    border: 2px solid #1976D2;
}
)";
}

} // namespace MaterialStyle

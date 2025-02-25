#include <QApplication>
#include "mainwindow.hpp"

void setup_dark_mode(QApplication &app);

// Program entry point
int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    app.setWindowIcon(QIcon("qrc:/images/icon.png"));

    // Check if the user wants to use light mode
    QString lightMode = "--light-mode";
    QString lightModeShort = "-l";
    if (argc > 1 && (lightMode.compare(argv[1]) == 0 || lightModeShort.compare(argv[1]) == 0)) {
        app.setStyle("Fusion");
    } else {
        app.setStyle("Fusion");
        setup_dark_mode(app);
    }

    MainWindow mainWindow;
    mainWindow.show();

    return app.exec();
}

/**
 * @brief Sets up the dark mode for the application.
 * 
 * This function sets the color palette for the application to achieve a dark mode appearance.
 * It sets the colors for the window, window text, base, alternate base, text, button, and button text.
 * 
 * @param app The QApplication object to set the palette for.
 */
void setup_dark_mode(QApplication &app) {
    QPalette palette;
    palette.setColor(QPalette::Window, QColor(53, 53, 53));
    palette.setColor(QPalette::WindowText, Qt::white);
    palette.setColor(QPalette::Base, QColor(25, 25, 25));
    palette.setColor(QPalette::AlternateBase, QColor(30, 30, 30));
    palette.setColor(QPalette::Text, Qt::white);
    palette.setColor(QPalette::Button, QColor(53, 53, 53));
    palette.setColor(QPalette::ButtonText, Qt::white);
    app.setPalette(palette);
}
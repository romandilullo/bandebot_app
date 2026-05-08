

struct GenericColor
{
    uint32_t rgb;

    GenericColor(uint8_t r, uint8_t g, uint8_t b) 
        : rgb((r << 16) | (g << 8) | (b << 0)) {}
    
    GenericColor(uint32_t color) : rgb(color) {}

    uint8_t getRed() const { return (rgb >> 16) & 0xFF; }
    uint8_t getGreen() const { return (rgb >> 8) & 0xFF; }
    uint8_t getBlue() const { return rgb & 0xFF; }

    static const GenericColor RED;
    static const GenericColor GREEN;
    static const GenericColor BLUE;
    static const GenericColor WHITE;
    static const GenericColor YELLOW;
    static const GenericColor CYAN;
    static const GenericColor MAGENTA;
    static const GenericColor BLACK;
    static const GenericColor ORANGE;
    static const GenericColor LIGHT_GRAY;
    static const GenericColor LIGHT_BLUE;
};


const GenericColor GenericColor::RED(255, 0, 0);
const GenericColor GenericColor::GREEN(0, 255, 0);
const GenericColor GenericColor::BLUE(0, 0, 255);
const GenericColor GenericColor::WHITE(255, 255, 255);
const GenericColor GenericColor::YELLOW(255, 255, 0);
const GenericColor GenericColor::CYAN(0, 255, 255);
const GenericColor GenericColor::MAGENTA(255, 0, 255);
const GenericColor GenericColor::BLACK(0, 0, 0);
const GenericColor GenericColor::ORANGE(255, 128, 0);
const GenericColor GenericColor::LIGHT_GRAY(200, 200, 200);
const GenericColor GenericColor::LIGHT_BLUE(173, 216, 230);
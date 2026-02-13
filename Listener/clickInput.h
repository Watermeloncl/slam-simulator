#ifndef LISTENER_CLICK_INPUT_H_
#define LISTENER_CLICK_INPUT_H_

class ClickInput {
private:
    bool clicked = false;
    int x = 0;
    int y = 0;
public:
    ClickInput();
    ~ClickInput();

    bool GetClicked();
    void SetClicked();
    void SetUnclicked();

    int GetX();
    void SetX(int x);

    int GetY();
    void SetY(int y);
};

#endif
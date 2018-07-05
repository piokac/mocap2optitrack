#ifndef TEXTTOUR3_H
#define TEXTTOUR3_H

#include <QMainWindow>
#include <UR3Intermediator.h>


namespace Ui {
class TextToUR3;
}

struct TextToUR3
{
  float x,y,z;
  QString text;
  char[64] znaki;
};
class TextToUR3 : public QMainWindow
{
    Q_OBJECT

public:
    explicit TextToUR3(QWidget *parent = 0);
    ~TextToUR3();
    void setCharacteristicPoint1();
    void setCharacteristicPoint2(); //2 funkcje które rozpoznają 2 charakterysyuczne punkty na klawiaturze (litera Q, P???)
    void setCharacteristicPointN(); //ewentualnie wiecej punktow charakterystycznych, w zaleznosci od tego ile bedzie w praktyce potrzebne
    void getText(QString text); //pobiera tekst z okna w module wizualizacji, ktory bedzie pozniej fizycznie wpisywany przez UR3 markerem
    char WyodrebnijLitere(QString text); // wyodrebnia kolejno 1,2,3..[lub tworzy tablice] litere aby pozniej funkcją mów dokonać przesunięcie w dany punkt
    void countDistanceBetween();//oblicza odstęp pomiędzy 2 charakterystycznymi punktami
    void move(); //wykonaj przesunięcie o dany wektor obliczany w oparciu o odległość od siebie punktór charakterystycznych
    void getCalibrationData(); //tłumaczy robotowi, jaki ruch w przestrzeni musi zrobić, aby przeskoczyć o pewien wektor, aby jednoczesnie nie zachaczyc
   // markerem o niechciany punkt
    void ReturnToDefult(); //kaze robotowi przerwac i powrocic do początkowego punktu zerowego

private:
    Ui::TextToUR3 *ui;
};

#endif // TEXTTOUR3_H

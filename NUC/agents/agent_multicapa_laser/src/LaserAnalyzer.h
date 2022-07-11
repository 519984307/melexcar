//
// Created by javier on 19/05/22.
//

#ifndef AGENT_THREE_CAMERAS_LASERANALYZER_H
#define AGENT_THREE_CAMERAS_LASERANALYZER_H

#include <Laser.h>
#include <vector>
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/zip.hpp>
#include <math.h>

#define NADA 0
#define SEG 1
#define ESQ 2
#define CIR 3
#define BKP 4

class LaserAnalyzer {

    private:
        std::vector<float> xcoords, ycoords, dists, angles, pendiente;
        std::vector<bool> breakpoints;
        int limiteParaBreakpooint;
        float maxLaser, limParaRecto;
        std::vector<int> analizarCurvatura();
        std::vector<bool> verSiHayAgo();
        bool esEsquina(int i);
        bool esSegmento(int i);


    public:
        LaserAnalyzer(RoboCompLaser::TLaserData laser, int limite, float maxLaser, float limParaRecto);
        void filtrarTipos(std::vector<std::tuple<float, float, int>> &data);
        void filtrarbptobp(std::vector<std::tuple<float, float, int>> &data);
        ~LaserAnalyzer();

        std::vector<float> calcularPendiente();
        std::vector<bool> extraerBreakPoints();
        std::vector<std::tuple<float, float, int>> extraerDatosParaPintar();
};


#endif //AGENT_THREE_CAMERAS_LASERANALYZER_H

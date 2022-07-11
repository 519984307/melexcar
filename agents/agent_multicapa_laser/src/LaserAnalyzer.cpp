//
// Created by javier on 19/05/22.
//

#include "LaserAnalyzer.h"

LaserAnalyzer::LaserAnalyzer(RoboCompLaser::TLaserData laser, int limiteParaBreakpooint, float maxLaser, float limParaRecto)
{
    this->limiteParaBreakpooint = limiteParaBreakpooint;
    this->maxLaser = maxLaser;
    this->limParaRecto = limParaRecto;

    for(auto &data: laser)
    {
        dists.push_back(data.dist);
        angles.push_back(data.angle);
        xcoords.push_back(data.dist * cos(data.angle - M_PI_2));
        ycoords.push_back(data.dist * sin(data.angle - M_PI_2));
    }

    //TODO: filtrar
}

LaserAnalyzer::~LaserAnalyzer()
{
    dists.clear();
    angles.clear();
    xcoords.clear();
    ycoords.clear();
}

std::vector<float> LaserAnalyzer::calcularPendiente()
{
    int depth = 4;
    for(auto&& dist: iter::sliding_window(dists, depth))
        pendiente.push_back(dist[depth-1] - dist[0]);
    pendiente.push_back(0); // Mantenemos tamaño de 180
    return pendiente;
}

std::vector<bool> LaserAnalyzer::extraerBreakPoints()
{
    breakpoints.push_back(1); // Mantenemos tamaño de 180
    bool bp = false;
    for(auto&& p: iter::sliding_window(dists, 3)){
//        if(bp == false){
            bp = abs(p[0] - p[1]) > limiteParaBreakpooint || abs(p[1] - p[2]) > limiteParaBreakpooint  ;
            breakpoints.push_back(bp);
//            if(bp == true)
//                breakpoints.push_back(true);
//        }
//        else
//            bp = false;
    }
    return breakpoints;
}

std::vector<int> LaserAnalyzer::analizarCurvatura()
{
    std::vector<int> tipos;
    int tipo;
    std::vector<bool> hayAlgo = verSiHayAgo();
    int offset = 0;
    for(int i = offset; i < angles.size(); i++)
    {
        if(hayAlgo[i])
        {
            if(breakpoints[i])   tipo = BKP;
            else if(esEsquina(i))       tipo = ESQ;
            else if(esSegmento(i))      tipo = SEG;
            else                        tipo = CIR;
        }
        else                            tipo = NADA;

        tipos.push_back(tipo);
    }
    return tipos;
}

bool LaserAnalyzer::esEsquina(int i)
{
    bool opuestosEnLados = (pendiente[i+1] < 0  &&  pendiente[i-1] > 0) || (pendiente[i+1] > 0  &&  pendiente[i-1] < 0);
    bool igualIzqda = (pendiente[i-2] > 0  &&  pendiente[i-1] > 0) || (pendiente[i-2] < 0  &&  pendiente[i-1] < 0);
    bool igualDcha = (pendiente[i] > 0  &&  pendiente[i+1] > 0) || (pendiente[i] < 0  &&  pendiente[i+1] < 0);
    bool igualIzqda2 = (pendiente[i-2] > 0  &&  pendiente[i-3] > 0) || (pendiente[i-2] < 0  &&  pendiente[i-3] < 0);
    bool igualDcha2 = (pendiente[i+2] > 0  &&  pendiente[i+1] > 0) || (pendiente[i+2] < 0  &&  pendiente[i+1] < 0);
    bool igualIzqda3 = (pendiente[i-3] > 0  &&  pendiente[i-4] > 0) || (pendiente[i-3] < 0  &&  pendiente[i-4] < 0);
    bool igualDcha3 = (pendiente[i+3] > 0  &&  pendiente[i+2] > 0) || (pendiente[i+3] < 0  &&  pendiente[i+2] < 0);

    return opuestosEnLados && igualDcha && igualIzqda && igualIzqda2 && igualDcha2 && igualIzqda3 && igualDcha3;
}

bool LaserAnalyzer::esSegmento(int i)
{
    bool mismoSigno = true, pocaVariacion = true;
    int v = 3;

    for (int j = 1; j < v; j++)
    {
        mismoSigno |= (pendiente[i] < 0  &&  pendiente[i+j] < 0) || (pendiente[i] > 0  &&  pendiente[i+j] > 0);
        mismoSigno |= (pendiente[i] < 0  &&  pendiente[i-j] < 0) || (pendiente[i] > 0  &&  pendiente[i-j] > 0);
    }


    for (int j = -5; j < v-1; j++)
    {
        pocaVariacion &= abs(pendiente[i+(j+1)] - pendiente[i+j]) < limParaRecto;
    }

    return  pocaVariacion && mismoSigno;
}

std::vector<bool> LaserAnalyzer::verSiHayAgo()
{
    std::vector<bool> hayAlgo;
    for(auto d: dists)
        hayAlgo.push_back(d < maxLaser);
    return hayAlgo;
}

std::vector<std::tuple<float, float, int>> LaserAnalyzer::extraerDatosParaPintar()
{
    std::vector<std::tuple<float, float, int>> datosParaPintar;
    std::vector<int> tipos = analizarCurvatura();
    //filtrarTipos(tipos);

    for (auto&& [x, y, t] : iter::zip(xcoords, ycoords, tipos))
        datosParaPintar.push_back(std::make_tuple(x, y, t));
    return datosParaPintar;
}

void LaserAnalyzer::filtrarTipos(std::vector<std::tuple<float, float, int>> &data)
{
    std::vector<int> tipos;
    for(auto &p: data) {
        auto &[x, y, type] = p;

        tipos.push_back(type);
    }
    int i=0;
        while(i<xcoords.size()){
            if (i>xcoords.size()-8){


                if (tipos[i] != tipos[i-1] && tipos[i] != tipos[i+1] && tipos[i] != 2 && tipos[i-1] != 2 && tipos[i] != 4 && tipos[i-1] != 4)
                    tipos [i] = tipos[i-1];
                if (tipos[i] != tipos[i-2] && tipos[i] != tipos[i+2] && tipos[i] != 2 && tipos[i-2] != 2 && tipos[i] != 4 && tipos[i-2] != 4)
                    tipos [i] = tipos[i-2];
                if (tipos[i] != tipos[i-3] && tipos[i] != tipos[i+3] && tipos[i] != 2 && tipos[i-3] != 2 && tipos[i] != 4 && tipos[i-3] != 4)
                    tipos [i] = tipos[i-3];
                if (tipos[i] != tipos[i-2] && tipos[i] != tipos[i+1] && tipos[i] != 2 && tipos[i-2] != 2 && tipos[i] != 4 && tipos[i-2] != 4)
                    tipos [i] = tipos[i-2];
                if (tipos[i] != tipos[i-2] && tipos[i] != tipos[i+3] && tipos[i] != 2  && tipos[i-2] != 2 && tipos[i] != 4  && tipos[i-2] != 4)
                    tipos [i] = tipos[i-2];
                if (tipos[i] != tipos[i-3] && tipos[i] != tipos[i+2] && tipos[i] != 2 && tipos[i-3] != 2 && tipos[i] != 4 && tipos[i-3] != 4)
                    tipos [i] = tipos[i-3];
                if (tipos[i] != tipos[i-3] && tipos[i] != tipos[i+1] && tipos[i] != 2 && tipos[i-3] != 2 && tipos[i] != 4 && tipos[i-3] != 4)
                    tipos [i] = tipos[i-3];
                if (tipos[i] != tipos[i-3] && tipos[i] != tipos[i-2] && tipos[i] != 2 && tipos[i-3] != 2 && tipos[i] != 4 && tipos[i-3] != 4)
                    tipos [i] = tipos[i-3];
            }else{
                if (tipos[i] != tipos[i-1] && tipos[i] != tipos[i+1] && tipos[i] != 2 && tipos[i+1] != 2 && tipos[i] != 4 && tipos[i+1] != 4)
                    tipos [i] = tipos[i+1];
                if (tipos[i] != tipos[i-2] && tipos[i] != tipos[i+2] && tipos[i] != 2  && tipos[i+2] != 2 && tipos[i] != 4 && tipos[i+2] != 4)
                    tipos [i] = tipos[i+2];
                if (tipos[i] != tipos[i-3] && tipos[i] != tipos[i+3] && tipos[i] != 2 && tipos[i+3] != 2 && tipos[i] != 4 && tipos[i+3] != 4)
                    tipos [i] = tipos[i+3];
                if (tipos[i] != tipos[i-2] && tipos[i] != tipos[i+1] && tipos[i] != 2  && tipos[i+1] != 2 && tipos[i] != 4 && tipos[i+1] != 4)
                    tipos [i] = tipos[i+1];
                if (tipos[i] != tipos[i-2] && tipos[i] != tipos[i+3] && tipos[i] != 2 && tipos[i+3] != 2 && tipos[i] != 4 && tipos[i+3] != 4)
                    tipos [i] = tipos[i+3];
                if (tipos[i] != tipos[i-3] && tipos[i] != tipos[i+2] && tipos[i] != 2 && tipos[i+2] != 2 && tipos[i] != 4 && tipos[i+2] != 4)
                    tipos [i] = tipos[i+2];
                if (tipos[i] != tipos[i-3] && tipos[i] != tipos[i+1] && tipos[i] != 2 && tipos[i+1] != 2 && tipos[i] != 4 && tipos[i+1] != 4)
                    tipos [i] = tipos[i+1];
            }
        i++;
        }
    i=0;
    for(auto &p: data) {
        auto &[x, y, type] = p;
        type=tipos[i];
        tipos.push_back(type);
        i++;
    }
}
void LaserAnalyzer::filtrarbptobp(std::vector<std::tuple<float, float, int>> &data){
    std::vector<int> tipos;
    for(auto &p: data) {
        auto &[x, y, type] = p;

        tipos.push_back(type);
    }

    int last = 0;
    std::vector<int> contadorTipos{0, 0, 0, 0, 0};
    for(int i = 0; i<xcoords.size(); i++)
    {
        if (tipos[i] == ESQ || tipos[i] == BKP){
            int val = std::max_element(contadorTipos.begin(), contadorTipos.end()) - contadorTipos.begin();
            for (int j = last+1; j < i; j++)
            {
                tipos[j] = val;
            }
            contadorTipos = {0, 0, 0, 0, 0};
            last = i;
        }
        else
        {
            contadorTipos[tipos[i]]++;
        }
    }
    int xi=0;
    for(auto &p: data) {
        auto &[x, y, type] = p;
        type=tipos[xi];

        xi++;
    }
}
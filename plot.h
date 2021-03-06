/**
 * @file plot.h
 * @author Aswath Muthuselvam (aswath@umd.edu)
 * @brief Plotting utility file.
 * @version 1.0
 * @date 02-15-2022
 * @copyright BSD3 Copyright (c) 2022
 
   Copyright (c) 2022, Aswath Muthuselvam
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.

   * Neither the name of easyRNG nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.
     
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
   FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
   DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
   SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
   OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

class Plot{
    public:
        Gnuplot gp;

        Plot(int width, int height){
            gp<<"set xrange [0:"<<width<<"]\n";
            gp<<"set yrange [0:"<<height<<"]\n";
            gp<<"set zrange [0:100]\n";
            gp<<"set xtics 1\n";
            gp<<"set ytics 1\n";
            gp<<"set ztics 1\n";
            gp<<"set grid\n";
            gp<<"set format x \"\"\n";
            gp<<"set format y \"\"\n"; 
            gp<<"set format z \"\"\n";           
        }

        Plot& xlabel(std::string xlabel){
            gp << "set xlabel '"<<xlabel<<"'\n";
            return *this;
        }
        Plot& ylabel(std::string ylabel){
            gp << "set ylabel '"<<ylabel<<"'\n";
            return *this;
        }
        Plot& zlabel(std::string zlabel){
            gp << "set zlabel '"<<zlabel<<"'\n";
            return *this;
        }
        Plot& setTitle(std::string title){
            gp << "set title '"<<title<<"'\n";
            return *this;
        }
        Plot& setGrid(bool grid){
            if(grid){
                gp<<"set grid\n";
            }else{
                gp<<"unset grid\n";
            }
            return *this;
        }

        void resetSession(){
            gp << "reset session";
        }

        void linePlot(std::vector<double>& v){
            gp << "plot '-' with lines title 'v0'\n";
            gp.send(v);
        }

        void drawNode(std::vector<boost::tuple<double, double>>& v){
            gp << "plot '-' using 1:2 with points pt 7\n";
            gp.send1d(v);
        }

        void drawConnection(std::vector<boost::tuple<double, double, double, double>>& v){
            gp << "plot '-' using 1:2:3:4 with vector notitle\n";
            gp.send1d(v);
        }

        void drawPath(std::vector<boost::tuple<double, double, double, double>>& v){
            gp << "plot '-' using 1:2:3:4 with vector head filled lt 2 lw 3 lc 2 title \"Path\"\n";
            gp.send1d(v);
        }

        void drawGraph(nodeT& start,
                            nodeT& goal,
                            nodeT& nodes, 
                            connectionT& connections,
                            obstacleT& obstacles,
                            connectionT& path
        ){
            gp << "plot '-' using 1:2 with points pt 35 ps 3 title \"Start\", \
            '-' using 1:2 with points pt 35 ps 3 title \"goal\", \
             '-' using 1:2 with points pt 7 title \"Nodes\", \
             '-' using 1:2:3:4 with vector title \"Connections\", \
             '-' using 1:2:3 with circles fillstyle pattern 4 transparent lc rgb '#990000' title \"Obstacles\", \
             '-' using 1:2:3:4 with vector head filled lt 2 lw 3 lc 2 title \"Path\"\n";

            gp.send1d(start);
            gp.send1d(goal);
            gp.send1d(nodes);
            gp.send1d(connections);
            gp.send1d(obstacles);
            gp.send1d(path);
        }

        void drawGraph(nodeT& start,
                    nodeT& goal,
                    nodeT& nodes, 
                    connectionT& connections,
                    obstacleT& obstacles
        ){
            gp << "plot '-' using 1:2 with points pt 35 ps 3 title \"Start\", \
            '-' using 1:2 with points pt 35 ps 3 title \"goal\", \
             '-' using 1:2 with points pt 7 title \"Nodes\", \
             '-' using 1:2:3:4 with vector title \"Connections\", \
             '-' using 1:2:3 with circles fillstyle pattern 4 transparent lc rgb '#990000' title \"Obstacles\"\n";

            gp.send1d(start);
            gp.send1d(goal);
            gp.send1d(nodes);
            gp.send1d(connections);
            gp.send1d(obstacles);
        }

        // void drawGraph3D(nodeT& start,
        //                     nodeT& goal,
        //                     nodeT& nodes, 
        //                     connectionT& connections,
        //                    obstacleT& obstacles
        // ){
        //     gp<<"set parametric\n";
        //     gp<<"set urange [0:2*pi]\n";
        //     gp<<"set vrange [-pi/2:pi/2]\n";
        //     gp<<"fx(x,r,v,u) = x + r*cos(v)*cos(u)\n";
        //     gp<<"fy(y,r,v,u) = y + r*cos(v)*sin(u)\n";
        //     gp<<"fz(z,r,v)   = z + r*sin(v)\n";
        //     gp<<"set xrange [-5:50]\n";
        //     gp<<"set yrange [-5:50]\n";
        //     gp<<"set zrange [-5:50]\n";
        //     gp<<"set hidden3d\n";
        //     gp<<"set size square\n";
        //     setTitle("RRT").xlabel("X").ylabel("Y").zlabel("Z");
        //     gp<<"x="<<0<<"\n";
        //     gp<<"y="<<0<<"\n";
        //     gp<<"z="<<0<<"\n";
        //     gp<<"r="<<0<<"\n";
        //     gp<<"splot fx(x,r,v,u),fy(y,r,v,u),fz(z,r,v) notitle\n";
        //     for(auto obstacle: obstacles){
        //         gp<<"replot fx("<<obstacle.get<0>()<<","<<obstacle.get<3>()<<",v,u),fy("
        //         <<obstacle.get<1>()<<","<<obstacle.get<3>()<<",v,u),fz("
        //         <<obstacle.get<2>()<<","<<obstacle.get<3>()<<",v) notitle\n";
        //         // gp<<"pause 1\n";
        //     }
            
        //     gp<<"replot '-' using 1:2:3 with points pt 35 ps 3 title \"Start\", \
        //     '-' using 1:2:3 with points pt 35 ps 3 title \"goal\", \
        //     '-' using 1:2:3 with points pt 7 title \"Nodes\", \
        //     '-' using 1:2:3:4:5:6 with vector title \"Connections\"\n";
        //     //'-' using 1:2:3:4:5 with points pt 7 ps 10 title \"Obstacles\"\n";

        //     gp.send1d(start);
        //     gp.send1d(goal);
        //     gp.send1d(nodes);
        //     gp.send1d(connections);
        //     gp<<"pause 0.3\n";
 
        // }

        void drawObstacle(std::vector<boost::tuple<double, double, double>>& v){
            gp << "plot '-' using 1:2:3 with circles notitle\n";
            gp.send1d(v);
        }

        void drawSphere(double x,double y,double r){
            // For 3D Spherical Obstacle
            gp << "x = "<<x<<"\n";
            gp << "y = "<<y<<"\n";
            gp << "r = "<<r<<"\n";
            gp << "set urange [0:2*pi]\n";
            gp << "set vrange [-pi/2:pi/2]\n";
            gp << "fx(x,v,u) = x + r*cos(v)*cos(u)\n";
            gp << "fy(y,v,u) = y + r*cos(v)*sin(u)\n";
            gp << "fz(z,v)   = z + r*sin(v)\n";
            gp << "plot fx(v,u),fy(v,u),fz(v)\n";
        }

};
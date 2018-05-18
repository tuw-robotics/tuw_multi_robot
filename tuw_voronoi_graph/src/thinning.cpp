#include <tuw_voronoi_map/thinning.h>
#include <ros/ros.h>

namespace voronoi_map
{

    void sceletonize_iteration(cv::Mat& img, int iter)
    {
        CV_Assert(img.channels() == 1);
        CV_Assert(img.depth() != sizeof(uchar));
        CV_Assert(img.rows > 3 && img.cols > 3);

        cv::Mat marker = cv::Mat::zeros(img.size(), CV_8UC1);

        int nRows = img.rows;
        int nCols = img.cols;

        if(img.isContinuous())
        {
            nCols *= nRows;
            nRows = 1;
        }

        int x, y;
        uchar *pAbove;
        uchar *pCurr;
        uchar *pBelow;
        uchar *nw, *no, *ne;    // north (pAbove)
        uchar *we, *me, *ea;
        uchar *sw, *so, *se;    // south (pBelow)

        uchar *pDst;

        // initialize row pointers
        pAbove = NULL;
        pCurr  = img.ptr<uchar>(0);
        pBelow = img.ptr<uchar>(1);

        for(y = 1; y < img.rows - 1; ++y)
        {
            // shift the rows up by one
            pAbove = pCurr;
            pCurr  = pBelow;
            pBelow = img.ptr<uchar>(y + 1);

            pDst = marker.ptr<uchar>(y);

            // initialize col pointers
            no = &(pAbove[0]);
            ne = &(pAbove[1]);
            me = &(pCurr[0]);
            ea = &(pCurr[1]);
            so = &(pBelow[0]);
            se = &(pBelow[1]);

            for(x = 1; x < img.cols - 1; ++x)
            {
                // shift col pointers left by one (scan left to right)
                nw = no;
                no = ne;
                ne = &(pAbove[x + 1]);
                we = me;
                me = ea;
                ea = &(pCurr[x + 1]);
                sw = so;
                so = se;
                se = &(pBelow[x + 1]);

                int A  = (*no == 0 && *ne == 1) + (*ne == 0 && *ea == 1) +
                         (*ea == 0 && *se == 1) + (*se == 0 && *so == 1) +
                         (*so == 0 && *sw == 1) + (*sw == 0 && *we == 1) +
                         (*we == 0 && *nw == 1) + (*nw == 0 && *no == 1);
                int B  = *no + *ne + *ea + *se + *so + *sw + *we + *nw;
                int m1 = iter == 0 ? (*no * *ea * *so) : (*no * *ea * *we);
                int m2 = iter == 0 ? (*ea * *so * *we) : (*no * *so * *we);

                if(A == 1 && (B >= 2 && B <= 6) && m1 == 0 && m2 == 0)
                    pDst[x] = 1;
            }
        }

        img &= ~marker;
    }

    void sceletonize(const cv::Mat& src, cv::Mat& dst)
    {
        dst = src.clone();
        dst /= 255;         // convert to binary image

        cv::Mat prev = cv::Mat::zeros(dst.size(), CV_8UC1);
        cv::Mat diff;

        do
        {
            sceletonize_iteration(dst, 0);
            sceletonize_iteration(dst, 1);
            cv::absdiff(dst, prev, diff);
            dst.copyTo(prev);
        }
        while(cv::countNonZero(diff) > 0);

        dst *= 255;
    }


    std::queue<Index> q;


    void greyscale_thinning(const cv::Mat& src, cv::Mat& dst)
    {
        int initCosts = 2;
        int pathCosts = 1;

        for(int i = 1; i < dst.rows - 1; i++)
        {
            for(int j = 1; j < dst.cols - 1; j++)
            {
                float pAct = src.at<float_t>(i, j);

                //skip unknown area
                if(pAct > 0 && i > 0 && i < src.rows - 1 && j > 0 && j < src.cols - 1)
                {
                    //Find neighbour size
                    float p[8] =
                    {
                        src.at<float_t>(i, j - 1),
                        src.at<float_t>(i - 1, j - 1),
                        src.at<float_t>(i - 1, j),
                        src.at<float_t>(i - 1, j + 1),
                        src.at<float_t>(i, j + 1),
                        src.at<float_t>(i + 1, j + 1),
                        src.at<float_t>(i + 1, j),
                        src.at<float_t>(i + 1, j - 1)
                    };

                    bool bigger[8] = {p[0] > pAct, p[1] > pAct, p[2] > pAct, p[3] > pAct, p[4] > pAct, p[5] > pAct, p[6] > pAct, p[7] > pAct};
                    bool smaller[8] = {p[0] < pAct, p[1] < pAct, p[2] < pAct, p[3] < pAct, p[4] < pAct, p[5] < pAct, p[6] < pAct, p[7] < pAct};




                    //Find line Saddle points
                    if(bigger[4] && bigger[0] && !bigger[2] && !bigger[6])
                    {
                        //straight saddlepoint
                        //    -
                        //  + o +
                        //    -
                        q.push(Index(i, j, pAct));
                        dst.at<int8_t>(i, j) = initCosts;
                    }
                    else if(!bigger[4] && !bigger[0] && bigger[2] && bigger[6])
                    {
                        //straight saddlepoint
                        //    +
                        //  - o -
                        //    +
                        q.push(Index(i, j, pAct));
                        dst.at<int8_t>(i, j) = initCosts;
                    }
                    else if(bigger[1] && bigger[5] && !(bigger[2] && bigger[3] && bigger[4]) && !(bigger[0] && bigger[7] && bigger[6]))
                    {
                        //diagonal saddlepoint
                        //  + - -
                        //  - o -
                        //  - - +
                        q.push(Index(i, j, pAct));
                        dst.at<int8_t>(i, j) = initCosts;
                    }
                    else if(bigger[3] && bigger[7] && !(bigger[2] && bigger[1] && bigger[0]) && !(bigger[4] && bigger[5] && bigger[6]))
                    {
                        //diagonal saddlepoint
                        //  - - +
                        //  - o -
                        //  + - -
                        q.push(Index(i, j, pAct));
                        dst.at<int8_t>(i, j) = initCosts;
                    }

                    //Find additional saddle points

                    //    - - +   + - -
                    //    - o -   - o -
                    //      +   + ....

                    else if(!bigger[0] && !bigger[1] && !bigger[2] && bigger[3] && !bigger[4] && bigger[6])
                    {
                        q.push(Index(i, j, pAct));
                        dst.at<int8_t>(i, j) = initCosts;
                    }
                    else if(!bigger[0] && bigger[1] && !bigger[2] && !bigger[3] && !bigger[4] && bigger[6])
                    {
                        q.push(Index(i, j, pAct));
                        dst.at<int8_t>(i, j) = initCosts;
                    }

                    else if(!bigger[0] && bigger[2] && !bigger[4] && bigger[5] && !bigger[6] && !bigger[7])
                    {
                        q.push(Index(i, j, pAct));
                        dst.at<int8_t>(i, j) = initCosts;
                    }
                    else if(!bigger[0] && bigger[2] && !bigger[4] && !bigger[5] && !bigger[6] && bigger[7])
                    {
                        q.push(Index(i, j, pAct));
                        dst.at<int8_t>(i, j) = initCosts;
                    }

                    else if(!bigger[0] && bigger[1] && !bigger[2] && bigger[4] && !bigger[6] && !bigger[7])
                    {
                        q.push(Index(i, j, pAct));
                        dst.at<int8_t>(i, j) = initCosts;
                    }
                    else if(!bigger[0] && !bigger[1] && !bigger[2] && bigger[4] && !bigger[6] && bigger[7])
                    {
                        q.push(Index(i, j, pAct));
                        dst.at<int8_t>(i, j) = initCosts;
                    }

                    else if(bigger[0] && !bigger[2] && bigger[3] && !bigger[4] && !bigger[5] && !bigger[6])
                    {
                        q.push(Index(i, j, pAct));
                        dst.at<int8_t>(i, j) = initCosts;
                    }
                    else if(bigger[0] && !bigger[2] && !bigger[3] && !bigger[4] && bigger[5] && !bigger[6])
                    {
                        q.push(Index(i, j, pAct));
                        dst.at<int8_t>(i, j) = initCosts;
                    }
                    //Maxima
                    else if(!bigger[0] && !bigger[1] && !bigger[2] && !bigger[3] && !bigger[4] && !bigger[5] && !bigger[6] && !bigger[7])
                    {
                        dst.at<int8_t>(i, j) = initCosts;
                        q.push(Index(i, j, pAct));


                    }

                    //Find Edges


                    if(smaller[0] && smaller[1] &&  bigger[3] && smaller[5] && smaller[6] && smaller[7])
                    {
                        //left top edge
                        //     -   +
                        //     - o
                        //     - - -

                        q.push(Index(i, j, pAct));
                        dst.at<int8_t>(i, j) = initCosts;
                    }
                    else if(smaller[1] && smaller[2] && smaller[3] && smaller[4] && smaller[5] && bigger[7])
                    {
                        //left bottom edge
                        //     - - -
                        //       o -
                        //     +   -

                        q.push(Index(i, j, pAct));
                        dst.at<int8_t>(i, j) = initCosts;
                    }
                    else if(bigger[1] && smaller[3] && smaller[4] && smaller[5] && smaller[6] && smaller[7])
                    {
                        //right top edge
                        //     +   -
                        //       o -
                        //     - - -

                        q.push(Index(i, j, pAct));
                        dst.at<int8_t>(i, j) = initCosts;
                    }
                    else if(smaller[0] && smaller[1] && smaller[2] && smaller[3] &&  bigger[5] && smaller[7])
                    {
                        //right bottom edge
                        //     - - -
                        //     - o
                        //     -   +

                        q.push(Index(i, j, pAct));
                        dst.at<int8_t>(i, j) = initCosts;
                    }




                    if(pAct > 1 && i > 1 && j > 1 && i < src.rows - 2 && j < src.cols - 2)   //Dont look for duplex saddle points at the edges of the map
                    {

                        //Find duplex saddle points
                        if((!smaller[5] || !smaller[6] || !smaller[4]) && !bigger[7]  && !bigger[3])
                        {
                            //Diagonal duplex saddle Point
                            //  + + -
                            //  + o   -
                            //  -   o +
                            //    - + +

                            int m = i - 1;
                            int n = j - 1;

                            float pot = src.at<float_t>(m, n);

                            float pp2[8] =
                            {
                                src.at<float_t>(m, n - 1),
                                src.at<float_t>(m - 1, n - 1),
                                src.at<float_t>(m - 1, n),
                                src.at<float_t>(m - 1, n + 1),
                                src.at<float_t>(m, n + 1),
                                src.at<float_t>(m + 1, n + 1),
                                src.at<float_t>(m + 1, n),
                                src.at<float_t>(m + 1, n - 1)
                            };
                            bool bigger2[8] = {pp2[0] > pot, pp2[1] > pot, pp2[2] > pot, pp2[3] > pot, pp2[4] > pot, pp2[5] > pot, pp2[6] > pot, pp2[7] > pot};
                            bool smaller2[8] = {pp2[0] < pot, pp2[1] < pot, pp2[2] < pot, pp2[3] < pot, pp2[4] < pot, pp2[5] < pot, pp2[6] < pot, pp2[7] < pot};

                            if((!smaller2[0] || !smaller2[1] || !smaller2[2]) && !bigger2[3] && !bigger2[7] && pot > 1)
                            {
                                q.push(Index(i, j, pAct));
                                dst.at<int8_t>(i, j) = initCosts;

                                q.push(Index(m, n, pAct));
                                dst.at<int8_t>(m, n) = initCosts;
                            }
                        }

                        if((!smaller[2] || !smaller[3] || !smaller[4]) &&  !bigger[1] && !bigger[5])
                        {
                            //Diagonal duplex saddle Point
                            //    - + +
                            //  -   o +
                            //  + o   -
                            //  + + -

                            int m = i + 1;
                            int n = j - 1;
                            float pot = src.at<float_t>(m, n);

                            float pp2[8] =
                            {
                                src.at<float_t>(m, n - 1),
                                src.at<float_t>(m - 1, n - 1),
                                src.at<float_t>(m - 1, n),
                                src.at<float_t>(m - 1, n + 1),
                                src.at<float_t>(m, n + 1),
                                src.at<float_t>(m + 1, n + 1),
                                src.at<float_t>(m + 1, n),
                                src.at<float_t>(m + 1, n - 1)
                            };
                            bool bigger2[8] = {pp2[0] > pot, pp2[1] > pot, pp2[2] > pot, pp2[3] > pot, pp2[4] > pot, pp2[5] > pot, pp2[6] > pot, pp2[7] > pot};
                            bool smaller2[8] = {pp2[0] < pot, pp2[1] < pot, pp2[2] < pot, pp2[3] < pot, pp2[4] < pot, pp2[5] < pot, pp2[6] < pot, pp2[7] < pot};

                            if((!smaller2[0] || !smaller2[7] || !smaller2[6]) && !bigger2[1] && !bigger2[5] && pot > 1)
                            {
                                q.push(Index(i, j, pAct));
                                dst.at<int8_t>(i, j) = initCosts;

                                q.push(Index(m, n, pAct));
                                dst.at<int8_t>(m, n) = initCosts;
                            }
                        }

                        if((!smaller[2] || !smaller[3] || !smaller[1]) && !bigger[0] && !bigger[4])
                        {
                            //Straight duplex saddle Point

                            //  + + +
                            //  - o -
                            //  - o -
                            //  + + +

                            int m = i + 1;
                            int n = j;

                            float pot = src.at<float_t>(m, n);


                            float pp2[8] =
                            {
                                src.at<float_t>(m, n - 1),
                                src.at<float_t>(m - 1, n - 1),
                                src.at<float_t>(m - 1, n),
                                src.at<float_t>(m - 1, n + 1),
                                src.at<float_t>(m, n + 1),
                                src.at<float_t>(m + 1, n + 1),
                                src.at<float_t>(m + 1, n),
                                src.at<float_t>(m + 1, n - 1)
                            };


                            bool bigger2[8] = {pp2[0] > pot, pp2[1] > pot, pp2[2] > pot, pp2[3] > pot, pp2[4] > pot, pp2[5] > pot, pp2[6] > pot, pp2[7] > pot};
                            bool smaller2[8] = {pp2[0] < pot, pp2[1] < pot, pp2[2] < pot, pp2[3] < pot, pp2[4] < pot, pp2[5] < pot, pp2[6] < pot, pp2[7] < pot};

                            if((!smaller2[5] || !smaller2[6] || !smaller2[7]) && !bigger2[0] && !bigger2[4]  && pot > 1)
                            {
                                q.push(Index(i, j, pAct));
                                dst.at<int8_t>(i, j) = initCosts;

                                q.push(Index(m, n, pAct));
                                dst.at<int8_t>(m, n) = initCosts;
                            }
                        }

                        if((!smaller[7] || !smaller[0] || !smaller[1]) && !bigger[2]  && !bigger[6])
                        {
                            //Straight duplex saddle Point

                            //  + - - +
                            //  + o o +
                            //  + - - +

                            int m = i;
                            int n = j + 1;
                            float pot = src.at<float_t>(m, n);

                            float pp2[8] =
                            {
                                src.at<float_t>(m, n - 1),
                                src.at<float_t>(m - 1, n - 1),
                                src.at<float_t>(m - 1, n),
                                src.at<float_t>(m - 1, n + 1),
                                src.at<float_t>(m, n + 1),
                                src.at<float_t>(m + 1, n + 1),
                                src.at<float_t>(m + 1, n),
                                src.at<float_t>(m + 1, n - 1)
                            };
                            bool bigger2[8] = {pp2[0] > pot, pp2[1] > pot, pp2[2] > pot, pp2[3] > pot, pp2[4] > pot, pp2[5] > pot, pp2[6] > pot, pp2[7] > pot};
                            bool smaller2[8] = {pp2[0] < pot, pp2[1] < pot, pp2[2] < pot, pp2[3] < pot, pp2[4] < pot, pp2[5] < pot, pp2[6] < pot, pp2[7] < pot};

                            if((!smaller2[3] || !smaller2[4] || !smaller2[5]) && !bigger2[2]  && !bigger2[6] && pot > 1)
                            {
                                q.push(Index(i, j, pAct));
                                dst.at<int8_t>(i, j) = initCosts;

                                q.push(Index(m, n, pAct));
                                dst.at<int8_t>(m, n) = initCosts;
                            }
                        }
                    }

                }
            }
        }

        while(!q.empty())
        {
            Index pixel = q.front();
            q.pop();

            int costs = dst.at<uint8_t>(pixel.i, pixel.j);
            Index next = getMaximumNeighbour(pixel.i, pixel.j, src, dst);

            if(next.i >= 0 && next.j >= 0 && dst.at<uint8_t>(next.i, next.j) == 0)
            {
                if(src.at<float_t>(next.i, next.j) > 0)
                {
                    dst.at<uint8_t>(next.i, next.j) = costs + pathCosts;
                    q.push(next);
                }
            }
        }
    }

    Index getMaximumNeighbour(int i, int j, const cv::Mat& src, cv::Mat& dst)
    {
        //Return on boarder
        if(i == 0 || i == src.rows - 1 || j == 0 || j == src.cols - 1)
            return Index(-1, -1, -1);

        int costs = dst.at<uint8_t>(i, j);
        Index p[8] =
        {
            Index(i, j - 1, src.at<float_t>(i, j - 1)),
            Index(i - 1, j - 1, src.at<float_t>(i - 1, j - 1)),
            Index(i - 1, j, src.at<float_t>(i - 1, j)),
            Index(i - 1, j + 1, src.at<float_t>(i - 1, j + 1)),
            Index(i, j + 1, src.at<float_t>(i, j + 1)),
            Index(i + 1, j + 1, src.at<float_t>(i + 1, j + 1)),
            Index(i + 1, j, src.at<float_t>(i + 1, j)),
            Index(i + 1, j - 1, src.at<float_t>(i + 1, j - 1))
        };

        Index act = Index(-1, -1, -1);

        for(int i = 0; i < 8; i++)
        {
            if(p[i].potential > act.potential)
            {
                int actCosts = dst.at<uint8_t>(p[i].i, p[i].j);

                if(actCosts != costs)
                {
                    act = p[i];
                }

            }
        }


        return act;
    }



}


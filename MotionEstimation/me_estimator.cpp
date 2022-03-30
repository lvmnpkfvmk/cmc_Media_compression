#include "me_estimator.h"
#include <unordered_set>
#include "metric.h"
#include <iostream>
#include <algorithm>

MotionEstimator::MotionEstimator(size_t width, size_t height, unsigned char quality, bool use_half_pixel)
    : width(width)
    , height(height)
    , quality(quality)
    , use_half_pixel(use_half_pixel)
    , width_ext(width + 2 * BORDER)
    , num_blocks_hor((width + BLOCK_SIZE - 1) / BLOCK_SIZE)
    , num_blocks_vert((height + BLOCK_SIZE - 1) / BLOCK_SIZE)
    , first_row_offset(width_ext * BORDER + BORDER)
    , me_field(num_blocks_hor, num_blocks_vert, BLOCK_SIZE)
    , prev_me_field(num_blocks_hor, num_blocks_vert, BLOCK_SIZE)
    , width_borders(width + 2 * BORDER)
    , frame_number(0)
    , height_borders(height + 2 * BORDER)
{
    cur_Y_borders = new unsigned char[width_borders * height_borders];
    prev_Y_borders = new unsigned char[width_borders * height_borders];
    if (use_half_pixel) {
        prev_Y_up_borders = new unsigned char[width_borders * height_borders];
        prev_Y_up_left_borders = new unsigned char[width_borders * height_borders];
        prev_Y_left_borders = new unsigned char[width_borders * height_borders];
    }
    // PUT YOUR CODE HERE
    
}

MotionEstimator::~MotionEstimator() {
    delete[] cur_Y_borders;
    delete[] prev_Y_borders;
    if (use_half_pixel) {
        delete[] prev_Y_up_borders;
        delete[] prev_Y_up_left_borders;
        delete[] prev_Y_left_borders;
    }
    // PUT YOUR CODE HERE
}

std::pair<uint32_t, std::pair<int, int>> argmin(uint32_t matrix[3][3], uint8_t size) {
    std::pair<int,int> tmp;
    uint32_t min = 0xffffffff;
    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            if (matrix[i][j] < min) {
                min = matrix[i][j];
                tmp.first = i;
                tmp.second = j;
            } 
        }
    }
    std::pair<uint32_t, std::pair<int, int>> ans;
    ans.first = min; ans.second = tmp;
    return ans;
}

enum 
{
    TOP_ROW_POS,
    LEFT_COL_POS,
    RIGHT_COL_POS,
    MID_POS,
    TOP_LEFT_POS,
    LOW,
    MEDIUM,
    HIGH,
};

struct coords
{
    int cx;
    int cy;

    bool operator==(const coords& other) const {
        if (cx == other.cx && cy == other.cy)
            return true;
        return false;
    };
};

struct context
{
    int type;
    MV left;
    MV topleft;
    MV top;
    MV topright;
    struct coords median;
    struct coords mean_vector;
    uint8_t medlen;
    unsigned min_error;
    unsigned max_error;
    unsigned mean_error;
    unsigned LMA;
    int zero_motion;
};

int median(int *x, int size)
{
    size_t n = size / 2;
    std::nth_element(x, x + n, x + size);
    return x[n];
}

struct coords medianvec(struct context cntxt)
{
    if (cntxt.type == RIGHT_COL_POS) {
        int xcords[3] = {cntxt.left.x, cntxt.topleft.x, cntxt.top.x};
        int ycords[3] = {cntxt.left.y, cntxt.topleft.y, cntxt.top.y};
        return {median(xcords, 3), median(ycords, 3)};
    } else if (cntxt.type == MID_POS) {
        int xcords[4] = {cntxt.left.x, cntxt.topleft.x, cntxt.top.x, cntxt.topright.x};
        int ycords[4] = {cntxt.left.y, cntxt.topleft.y, cntxt.top.y, cntxt.topright.y};
        return {median(xcords, 4), median(ycords, 4)};
    } else {
        return {0, 0};
    }
}

void dump_context(struct context x)
{
    char str[] = "type=%d\n"
        "left :: x=%d, y=%d, err=%d\n"
        "topleft :: x=%d, y=%d, err=%d\n"
        "top :: x=%d, y=%d, err=%d\n"
        "topright :: x=%d, y=%d, err=%d\n" 
        "median :: x=%d, y=%d, len=%d\n";
    printf(str, x.type, x.left.x, x.left.y, x.left.error,
            x.topleft.x, x.topleft.y, x.topleft.error,
            x.top.x, x.top.y, x.top.error,
            x.topright.x, x.topright.y, x.topright.error,
            x.median.cx, x.median.cy, x.medlen);
}

inline void compute_sdp(uint32_t errors[3][3],const unsigned char *cur,
        const unsigned char *prev, size_t width_ext, int size = 16)
{

    auto func = size == 8 ? GetErrorSAD_8x8 : GetErrorSAD_16x16;
    for (int m = 0; m < 3; ++m) {
        for (int n = 0; n < 3; ++n) {
            if ((m + n) % 2 == 1 || (m == 1 || n == 1)) {
                const auto comp = prev + (m - 1) * width_ext + n - 1;
                errors[m][n] = func(cur, comp, width_ext);
            } else {
                errors[m][n] = 0xffffffff;
            }
        }
    }
}

inline void compute_edp(uint32_t errors[3][3],const unsigned char *cur,
        const unsigned char *prev, size_t width_ext, int size = 16)
{
    auto func = size == 8 ? GetErrorSAD_8x8 : GetErrorSAD_16x16;
    for (int m = 0; m < 3; ++m) {
        for (int n = 0; n < 3; ++n) {
            if ((m + n) % 2 == 0) {
                const auto comp = prev + (m - 1) * width_ext + n - 1;
                errors[m][n] = func(cur, comp, width_ext);
            } else {
                errors[m][n] = 0xffffffff;
            }
        }
    }
}

inline void compute_ldp(uint32_t errors[3][3],const unsigned char *cur,
        const unsigned char *prev, size_t width_ext, int size = 16)
{
    auto func = size == 8 ? GetErrorSAD_8x8 : GetErrorSAD_16x16;
    for (int m = 0; m < 3; ++m) {
        for (int n = 0; n < 3; ++n) {
            if ((m + n) % 2 == 1 || (m == 1 || n == 1)) {
                const auto comp = prev + 2 * (m - 1) * width_ext + 2 * n - 2;
                errors[m][n] = func(cur, comp, width_ext);
            } else {
                errors[m][n] = 0xffffffff;
            }
        }
    }
}

unsigned l1(int x1, int y1, int x2, int y2)
{
    return abs(x1 - x2) + abs(y1 - y2);
}
inline unsigned near(MV &a, MV &b)
{
    return l1(a.x, a.y, b.x, b.y) <= 1;
}

struct context MotionEstimator::get_context(size_t i, size_t j, MEField& mvectors)
{
    struct context tmp = {0};
    tmp.zero_motion = 1;
    if (i == 0) {
        // текущий блок в верхней строке
        if (j == 0) {
            // левый нижний блок
            tmp.type = TOP_LEFT_POS;
            tmp.median = {0, 0};
            tmp.min_error = 0xffffffff;
            tmp.max_error = 0xffffffff;
            tmp.mean_error = 0xffffffff;
            tmp.mean_vector = {0,0};
            tmp.LMA = LOW;
        } else {
            // любой другой блок в строке
            tmp.type = TOP_ROW_POS;
            tmp.left = mvectors.get_mv(j - 1, i);
            tmp.median = {tmp.left.x, tmp.left.y};
            tmp.min_error = tmp.left.error;
            tmp.max_error = tmp.left.error;
            tmp.mean_error = tmp.left.error;
            tmp.mean_vector = {0, 0};
            tmp.LMA = LOW;
        }
    } else {
        if (j == 0) {
            // левый столбец
            tmp.type = LEFT_COL_POS;
            tmp.top = mvectors.get_mv(j, i - 1);
            tmp.topright = mvectors.get_mv(j + 1, i - 1);
            tmp.min_error = std::min({tmp.top.error, tmp.topright.error});
            tmp.median = {(tmp.top.x + tmp.topright.x) / 2, 
                            (tmp.top.y + tmp.topright.y) / 2};
            tmp.max_error = std::max({tmp.top.error, tmp.topright.error});
            tmp.mean_error = (tmp.top.error + tmp.topright.error) / 2;
            tmp.mean_vector = {(tmp.top.x + tmp.topright.x) / 2, (tmp.top.y + tmp.topright.y) / 2};
            unsigned arr[2] = {l1(tmp.top.x, tmp.top.y, tmp.mean_vector.cx, tmp.mean_vector.cy),
                l1(tmp.topright.x, tmp.topright.y, tmp.mean_vector.cx, tmp.mean_vector.cy)};
            unsigned L = *std::max_element(arr, arr + 2);
            tmp.LMA = L < 1 ? LOW : L > 4 ? HIGH : MEDIUM;
        } else if (j == num_blocks_hor - 1) {
            // правый столбец
            tmp.type = RIGHT_COL_POS;
            tmp.left = mvectors.get_mv(j - 1, i);
            tmp.topleft = mvectors.get_mv(j - 1, i - 1);
            tmp.top = mvectors.get_mv(j, i - 1);
            tmp.min_error = std::min({tmp.left.error, tmp.topleft.error, tmp.top.error});
            tmp.median = medianvec(tmp);
            tmp.max_error = std::max({tmp.left.error, tmp.topleft.error, tmp.top.error});
            tmp.mean_error = (tmp.top.error + tmp.topleft.error + tmp.left.error) / 3;

            tmp.mean_vector = {(tmp.top.x + tmp.topleft.x + tmp.left.x) / 3, (tmp.top.y + tmp.topleft.y + tmp.left.y) / 3};
            unsigned arr[3] = {l1(tmp.top.x, tmp.top.y, tmp.mean_vector.cx, tmp.mean_vector.cy),
                l1(tmp.left.x, tmp.left.y, tmp.mean_vector.cx, tmp.mean_vector.cy),
                l1(tmp.topleft.x, tmp.topleft.y, tmp.mean_vector.cx, tmp.mean_vector.cy)};
            unsigned L = *std::max_element(arr, arr + 3);
            tmp.LMA = L < 1 ? LOW : L > 4 ? HIGH : MEDIUM;
        } else {
            // центр кадра, большинство блоков будут здесь
            tmp.type = MID_POS;
            tmp.left = mvectors.get_mv(j - 1, i);
            tmp.topleft = mvectors.get_mv(j - 1, i - 1);
            tmp.top = mvectors.get_mv(j, i - 1);
            tmp.topright = mvectors.get_mv(j + 1, i - 1);
            tmp.min_error = std::min({tmp.left.error, 
                    tmp.topleft.error, tmp.top.error, tmp.topright.error});
            tmp.median = medianvec(tmp);
            tmp.max_error = std::max({tmp.left.error, 
                    tmp.topleft.error, tmp.top.error, tmp.topright.error});
            tmp.mean_error = (tmp.top.error + tmp.topleft.error +
                    tmp.left.error + tmp.topright.error) / 4;

            tmp.mean_vector = {(tmp.top.x + tmp.topleft.x + tmp.left.x + tmp.topright.x) / 4, (tmp.top.y + tmp.topleft.y + tmp.left.y + tmp.topright.y) / 4};
            unsigned arr[4] = {l1(tmp.top.x, tmp.top.y, tmp.mean_vector.cx, tmp.mean_vector.cy),
                l1(tmp.left.x, tmp.left.y, tmp.mean_vector.cx, tmp.mean_vector.cy),
                l1(tmp.topleft.x, tmp.topleft.y, tmp.mean_vector.cx, tmp.mean_vector.cy),
                l1(tmp.topright.x, tmp.topright.y, tmp.mean_vector.cx, tmp.mean_vector.cy)};
            unsigned L = *std::max_element(arr, arr + 4);
            tmp.LMA = L < 1 ? LOW : L > 4 ? HIGH : MEDIUM;
        }
    }
    uint8_t a = abs(tmp.median.cx), b = abs(tmp.median.cy);
    uint8_t medlen = a > b ? a : b;
    if (medlen > 0) tmp.zero_motion = 0;
    tmp.medlen = medlen;
    return tmp;
}

int get_k_factor(struct context tmp)
{
    int k;
    switch(tmp.LMA){
        case LOW: k = 2; break;
        case MEDIUM: k = 4; break;
        case HIGH: k = 1; break;
        default: k = 3; break;
    }
    return k;
}

inline int find_min_and_shift(uint32_t errors[3][3],
        size_t width_ext, unsigned long &y, unsigned long &x,
        int &shift_x, int &shift_y, unsigned &minerror, int step)
{
    auto ans = argmin(errors, 3);
    int shift_x_tmp, shift_y_tmp;
    shift_x_tmp = (ans.second.second - 1) * step;
    shift_y_tmp = (ans.second.first - 1) * step;
    minerror = ans.first;
    y = shift_y_tmp * width_ext + y;
    x = shift_x_tmp + x;
    shift_x += shift_x_tmp;
    shift_y += shift_y_tmp;
    return shift_x_tmp == 0 && shift_y_tmp == 0;
}

std::vector<struct coords> get_dots(struct context c)
{
    std::vector<struct coords> inv_dots;
    inv_dots.push_back({0, 0});
    switch(c.type) {
        case TOP_ROW_POS:
            if (!(c.left.x == 0 && c.left.y == 0))
                inv_dots.push_back({c.left.x, c.left.y});
            break;
        case LEFT_COL_POS:
            if (!(c.top.x == 0 && c.top.y == 0))
                inv_dots.push_back({c.top.x, c.top.y});
            if (!(c.topright.x == 0 && c.topright.y == 0) && 
                    !(near(c.topright, c.top)))
                inv_dots.push_back({c.topright.x, c.topright.y});
            break;
        case RIGHT_COL_POS:
            if (!(c.top.x == 0 && c.top.y == 0))
                inv_dots.push_back({c.top.x, c.top.y});

            if (!(c.topleft.x == 0 && c.topleft.y == 0) && 
                    !(c.topleft.x == c.top.x && c.topleft.y == c.top.y) )
                inv_dots.push_back({c.topleft.x, c.topleft.y});

            if (!(c.left.x == 0 && c.left.y == 0) && 
                    !(c.left.x == c.top.x && c.left.y == c.top.y) &&
                    !(c.left.x == c.topleft.x && c.left.y == c.topleft.y) )
                inv_dots.push_back({c.left.x, c.left.y});
            break;
        case MID_POS:
            if (!(c.top.x == 0 && c.top.y == 0))
                inv_dots.push_back({c.top.x, c.top.y});

            if (!(c.topleft.x == 0 && c.topleft.y == 0) && 
                    !(c.topleft.x == c.top.x && c.topleft.y == c.top.y) )
                inv_dots.push_back({c.topleft.x, c.topleft.y});

            if (!(c.left.x == 0 && c.left.y == 0) && 
                    !(c.left.x == c.top.x && c.left.y == c.top.y) &&
                    !(c.left.x == c.topleft.x && c.left.y == c.topleft.y) )
                inv_dots.push_back({c.left.x, c.left.y});

            if (!(c.topright.x == 0 && c.topright.y == 0) && 
                    !(c.topright.x == c.top.x && c.topright.y == c.top.y) &&
                    !(c.topright.x == c.topleft.x && c.topright.y == c.topleft.y) &&
                    !(c.topright.x == c.left.x && c.topright.y == c.left.y))
                inv_dots.push_back({c.topright.x, c.topright.y});
            break; 
        default:
            break;
    }
    return inv_dots;
}

MV find_best_vector(struct context c, unsigned  long vert_offset, unsigned long hor_offset,
        std::unordered_map<ShiftDir, const uint8_t*, ShiftDirHash> prev_map,
        size_t width_ext, unsigned T1, unsigned T2,
        const unsigned char *cur, std::vector<struct coords> inv_dots, int size = 16)
{
    auto func = size == 8 ? GetErrorSAD_8x8 : GetErrorSAD_16x16;
    MV best_vector;
    unsigned minerror;
    best_vector.error = std::numeric_limits<long>::max();
    for (const auto& prev_pair : prev_map) {
        int shift_x = 0, shift_y = 0;
        auto y = vert_offset, x = hor_offset; 
        const auto prev = prev_pair.second + x + y;
        const auto comp1 = prev + c.mean_vector.cy * width_ext + c.mean_vector.cx;
        minerror = func(cur, comp1, width_ext);
        if (minerror < best_vector.error) {
            best_vector.x = c.mean_vector.cx;
            best_vector.y = c.mean_vector.cy;
            best_vector.shift_dir = prev_pair.first;
            best_vector.error = minerror;
        }
        if (minerror < T1) {
            break;
        }
        if (c.LMA == MEDIUM || c.LMA == HIGH) {
            for (const auto tmp : inv_dots) {
                const auto pos = prev + tmp.cy * width_ext + tmp.cx;
                if ((minerror = func(cur, pos, width_ext)) < best_vector.error) {
                    best_vector.x = tmp.cx;
                    best_vector.y = tmp.cy;
                    best_vector.shift_dir = prev_pair.first;
                    best_vector.error = minerror;
                }
            }
        }
        y = best_vector.y * width_ext + y;
        x = best_vector.x + x;
        shift_x += best_vector.x;
        shift_y += best_vector.y;
        if (best_vector.error < T1) {
            break;
        }
        uint32_t errors[3][3];
        for (int m = 0; m < 3; m++) {
            for (int n = 0; n < 3; n ++) {
                errors[m][n] = 0xffffffff;
            }
        }
        int cnt = 0, k = get_k_factor(c);
        auto prev2 = prev_pair.second + x + y;
        while (1) {
            prev2 = prev_pair.second + x + y;
            if (!k) {
                while(1) {
                    compute_edp(errors, cur, prev2, width_ext, size);
                    int stop = find_min_and_shift(errors, width_ext, y, x, shift_x, shift_y, minerror, 1);
                    prev2 = prev_pair.second + x + y;
                    if (stop || minerror < T2) break;
                    compute_ldp(errors, cur, prev2, width_ext, size);
                    stop = find_min_and_shift(errors, width_ext, y, x, shift_x, shift_y, minerror, 2);
                    prev2 = prev_pair.second + x + y;
                    if (stop || minerror < T2) break;
                }
                k = 1000;
            }
            compute_sdp(errors, cur, prev2, width_ext, size);
            int stop = find_min_and_shift(errors, width_ext, y, x, shift_x, shift_y, minerror, 1);
            k--;
            if (stop || minerror < T2) break;
        }
        if (minerror < best_vector.error) {
            best_vector.x = shift_x;
            best_vector.y = shift_y;
            best_vector.shift_dir = prev_pair.first;
            best_vector.error = minerror;
        }
    }
    return best_vector;
}

void MotionEstimator::CEstimate(const unsigned char* cur_Y,
                               const unsigned char* prev_Y,
                               const uint8_t* prev_Y_up,
                               const uint8_t* prev_Y_left,
                               const uint8_t* prev_Y_upleft,
                               MEField& mvectors) {
    std::unordered_map<ShiftDir, const uint8_t*, ShiftDirHash> prev_map {
        { ShiftDir::NONE, prev_Y }
    };
    std::unordered_map<ShiftDir, const uint8_t*, ShiftDirHash> prev_mapx {
        { ShiftDir::NONE, prev_Y }
    };

    if (use_half_pixel) {
        prev_map.emplace(ShiftDir::UP, prev_Y_up);
        prev_map.emplace(ShiftDir::LEFT, prev_Y_left);
        prev_map.emplace(ShiftDir::UPLEFT, prev_Y_upleft);
    } 

    for (size_t i = 0; i < num_blocks_vert; ++i) {
        for (size_t j = 0; j < num_blocks_hor; ++j) {
            const auto hor_offset = j * BLOCK_SIZE;
            const auto vert_offset = first_row_offset + i * BLOCK_SIZE * width_ext;
            const auto cur = cur_Y + vert_offset + hor_offset;
            MV best_vector;
            best_vector.error = std::numeric_limits<long>::max();
            auto c = get_context(i, j, mvectors);
            auto inv_dots = get_dots(c);
            if (frame_number != 0) {
                MV tmp = prev_me_field.get_mv(j,i);
                inv_dots.push_back({tmp.x, tmp.y});
                if (i != num_blocks_vert - 1) {
                    tmp = prev_me_field.get_mv(j, i + 1);
                    inv_dots.push_back({tmp.x, tmp.y});
                }
            }
            //dump_context(context);
            unsigned T1 = c.zero_motion ? c.max_error : c.min_error, T2 = c.mean_error;
            T1 = T1 > 156 ? T1 : 156; T1 = T1 < 312 ? T1 : 312;
            T2 = T2 > 156 ? T2 : 156; T2 = T2 < 312 ? T2 : 312;
            best_vector = find_best_vector(c, vert_offset, hor_offset,
                    prev_mapx, width_ext, T1, T2, cur, inv_dots);
            if (best_vector.error > T1 * 2 && use_half_pixel) {
                best_vector = find_best_vector(c, vert_offset, hor_offset,
                        prev_map, width_ext, T1, T2, cur, inv_dots);
            }
            //std::cout << best_vector.error << "\n";
            if (best_vector.error > T1 * 3 && c.LMA >= MEDIUM) {
                best_vector.Split();
                T1 /= 2; T2 /= 2;
                unsigned minerror;
                for (int h = 0; h < 4; ++h) {
                    auto& subvector = best_vector.SubVector(h);
                    subvector.error = std::numeric_limits<long>::max();

                    const auto hor_offset = j * BLOCK_SIZE + ((h & 1) ? BLOCK_SIZE / 2 : 0);
                    const auto vert_offset = first_row_offset + (i * BLOCK_SIZE + ((h > 1) ? BLOCK_SIZE / 2 : 0)) * width_ext;
                    const auto cur = cur_Y + vert_offset + hor_offset;
                    for (const auto& prev_pair : prev_mapx) {
                        int shift_x = 0, shift_y = 0;
                        auto y = vert_offset, x = hor_offset; 
                        const auto prev = prev_pair.second + x + y;
                        const auto comp1 = prev + c.mean_vector.cy * width_ext + c.mean_vector.cx;
                        minerror = GetErrorSAD_8x8(cur, comp1, width_ext);
                        if (minerror < subvector.error) {
                            subvector.x = c.mean_vector.cx;
                            subvector.y = c.mean_vector.cy;
                            subvector.shift_dir = prev_pair.first;
                            subvector.error = minerror;
                        }
                        if (minerror < T1) break;
                        if (c.LMA == MEDIUM || c.LMA == HIGH) {
                            for (const auto tmp : inv_dots) {
                                const auto pos = prev + tmp.cy * width_ext + tmp.cx;
                                if ((minerror = GetErrorSAD_8x8(cur, pos, width_ext)) < subvector.error) {
                                    subvector.x = tmp.cx;
                                    subvector.y = tmp.cy;
                                    subvector.shift_dir = prev_pair.first;
                                    subvector.error = minerror;
                                }
                            }
                        }
                        y = subvector.y * width_ext + y;
                        x = subvector.x + x;
                        shift_x += subvector.x;
                        shift_y += subvector.y;
                        if (subvector.error < T1) {
                            break;
                        }
                        uint32_t errors[3][3];
                        for (int m = 0; m < 3; m++) {
                            for (int n = 0; n < 3; n ++) {
                                errors[m][n] = 0xffffffff;
                            }
                        }
                        int cnt = 0, k = get_k_factor(c);
                        auto prev2 = prev_pair.second + x + y;
                        while (1) {
                            prev2 = prev_pair.second + x + y;
                            if (!k) {
                                while(1) {
                                    compute_edp(errors, cur, prev2, width_ext, 8);
                                    int stop = find_min_and_shift(errors, width_ext, y, x, shift_x, shift_y, minerror, 1);
                                    prev2 = prev_pair.second + x + y;
                                    if (stop || minerror < T2) break;
                                    compute_ldp(errors, cur, prev2, width_ext, 8);
                                    stop = find_min_and_shift(errors, width_ext, y, x, shift_x, shift_y, minerror, 2);
                                    prev2 = prev_pair.second + x + y;
                                    if (stop || minerror < T2) break;
                                }
                                k = 1000;
                            }
                            compute_sdp(errors, cur, prev2, width_ext, 8);
                            int stop = find_min_and_shift(errors, width_ext, y, x, shift_x, shift_y, minerror, 1);
                            k--;
                            if (stop || minerror < T2) break;
                        }
                        if (minerror < subvector.error) {
                            subvector.x = shift_x;
                            subvector.y = shift_y;
                            subvector.shift_dir = prev_pair.first;
                            subvector.error = minerror;
                        }
                    }
                    if (subvector.error > T1 * 3)
                    for (const auto& prev_pair : prev_map) {
                        int shift_x = 0, shift_y = 0;
                        auto y = vert_offset, x = hor_offset; 
                        const auto prev = prev_pair.second + x + y;
                        const auto comp1 = prev + c.mean_vector.cy * width_ext + c.mean_vector.cx;
                        minerror = GetErrorSAD_8x8(cur, comp1, width_ext);
                        if (minerror < subvector.error) {
                            subvector.x = c.mean_vector.cx;
                            subvector.y = c.mean_vector.cy;
                            subvector.shift_dir = prev_pair.first;
                            subvector.error = minerror;
                        }
                        if (minerror < T1) break;
                        if (c.LMA == MEDIUM || c.LMA == HIGH) {
                            for (const auto tmp : inv_dots) {
                                const auto pos = prev + tmp.cy * width_ext + tmp.cx;
                                if ((minerror = GetErrorSAD_8x8(cur, pos, width_ext)) < subvector.error) {
                                    subvector.x = tmp.cx;
                                    subvector.y = tmp.cy;
                                    subvector.shift_dir = prev_pair.first;
                                    subvector.error = minerror;
                                }
                            }
                        }
                        y = subvector.y * width_ext + y;
                        x = subvector.x + x;
                        shift_x += subvector.x;
                        shift_y += subvector.y;
                        if (subvector.error < T1) {
                            break;
                        }
                        uint32_t errors[3][3];
                        for (int m = 0; m < 3; m++) {
                            for (int n = 0; n < 3; n ++) {
                                errors[m][n] = 0xffffffff;
                            }
                        }
                        int cnt = 0, k = get_k_factor(c);
                        auto prev2 = prev_pair.second + x + y;
                        while (1) {
                            prev2 = prev_pair.second + x + y;
                            if (!k) {
                                while(1) {
                                    compute_edp(errors, cur, prev2, width_ext, 8);
                                    int stop = find_min_and_shift(errors, width_ext, y, x, shift_x, shift_y, minerror, 1);
                                    prev2 = prev_pair.second + x + y;
                                    if (stop || minerror < T2) break;
                                    compute_ldp(errors, cur, prev2, width_ext, 8);
                                    stop = find_min_and_shift(errors, width_ext, y, x, shift_x, shift_y, minerror, 2);
                                    prev2 = prev_pair.second + x + y;
                                    if (stop || minerror < T2) break;
                                }
                                k = 1000;
                            }
                            compute_sdp(errors, cur, prev2, width_ext, 8);
                            int stop = find_min_and_shift(errors, width_ext, y, x, shift_x, shift_y, minerror, 1);
                            k--;
                            if (stop || minerror < T2) break;
                        }
                        if (minerror < subvector.error) {
                            subvector.x = shift_x;
                            subvector.y = shift_y;
                            subvector.shift_dir = prev_pair.first;
                            subvector.error = minerror;
                        }
                    }
                }

                if (best_vector.SubVector(0).error
                        + best_vector.SubVector(1).error
                        + best_vector.SubVector(2).error
                        + best_vector.SubVector(3).error > best_vector.error * 0.7
                   ) {
                    best_vector.Unsplit();
                }
            }
            /*if (frame_number != 0) {
                std::cout << frame_number << " ";
                std::cout << prev_me_field.get_mv(j, i).x << " " << prev_me_field.get_mv(j, i).y << " " << prev_me_field.get_mv(j, i).error << "\n";
            } */
            //std::cout << "best vector is " << best_vector.x << " " <<best_vector.y << " " <<best_vector.error << "\n";
            mvectors.set_mv(j, i, best_vector);
        }
    }
}

void extend_with_borders(
        unsigned char *input,
        unsigned char *output,
    size_t height,
    size_t width,
    size_t border_size
) {
    // Copy frame to center of new 
    size_t new_width = width + 2 * border_size;
    auto p_output = output + new_width * border_size + border_size;
    auto p_input = input;
    for (size_t y = 0; y < height; ++y, p_output += 2 * border_size) {
        for (size_t x = 0; x < width; ++x, ++p_output, ++p_input) {
            *p_output = *p_input;
        }
    }

    // Left and right borders.
    p_output = output + new_width * border_size;
    for (size_t y = 0; y < height; ++y) {
        memset(p_output, p_output[border_size], border_size);
        p_output += border_size + width;
        memset(p_output, p_output[-1], border_size);
        p_output += border_size;
    }

    // Top and bottom borders.
    p_output = output;
    auto p_output_reference_row = p_output + new_width * border_size;

    for (size_t y = 0; y < border_size; ++y) {
        memcpy(p_output, p_output_reference_row, new_width);
        p_output += new_width;
    }
    p_output = output + new_width * (height + border_size);
    p_output_reference_row = p_output_reference_row - new_width;

    for (size_t y = 0; y < border_size; ++y) {
        memcpy(p_output, p_output_reference_row, new_width);
        p_output += new_width;
    }
}

void generate_subpixel_arrays(
    unsigned char* input,
    unsigned char* output_up,
    unsigned char* output_left,
    unsigned char* output_up_left,
    size_t height,
    size_t width
) {
    for (size_t y = 0; y < height; ++y) {
        for (size_t x = 0; x < width; ++x) {
            size_t cur_pixel_pos = y * width + x;
            size_t left_pixel_pos = y * width + x - 1;
            size_t left_up_pixel_pos = (y - 1) * width + x - 1;
            size_t up_pixel_pos = (y - 1) * width + x;
            
            if (y > 0) {
                output_up[cur_pixel_pos] = (int(input[cur_pixel_pos]) + input[up_pixel_pos]) / 2;
            } else {
                output_up[cur_pixel_pos] = input[cur_pixel_pos];
            }
            if (x > 0) {
                output_left[cur_pixel_pos] = (int(input[cur_pixel_pos]) + input[left_pixel_pos]) / 2;
            } else {
                output_left[cur_pixel_pos] = input[cur_pixel_pos];
            }

            if (x > 0 && y > 0) {   
                output_up_left[cur_pixel_pos] = (
                        int(input[cur_pixel_pos]) + 
                        input[left_pixel_pos] + 
                        input[left_up_pixel_pos] + 
                        input[up_pixel_pos]
                ) / 4;
            } else if (y == 0) {
                output_up_left[cur_pixel_pos] = output_left[cur_pixel_pos];
            } else {
                output_up_left[cur_pixel_pos] = output_up[cur_pixel_pos];
            }
        }
    }   
}

MEField MotionEstimator::Estimate(
    py::array_t<unsigned char> cur_Y,
    py::array_t<unsigned char> prev_Y
) {
    
    extend_with_borders((unsigned char *)cur_Y.request().ptr, cur_Y_borders, height, width, BORDER);
    extend_with_borders((unsigned char *)prev_Y.request().ptr, prev_Y_borders, height, width, BORDER);
    
    if (cur_Y.size() != prev_Y.size()){
        throw std::runtime_error("Input shapes must match");
    }
    
    if (use_half_pixel) {
        generate_subpixel_arrays(
            prev_Y_borders,
            prev_Y_up_borders,
            prev_Y_left_borders,
            prev_Y_up_left_borders,
            width_borders,
            height_borders
        );
    }

    MotionEstimator::CEstimate(
        cur_Y_borders,
        prev_Y_borders,
        prev_Y_up_borders,
        prev_Y_left_borders,
        prev_Y_up_left_borders,
        me_field
    );
    frame_number++;
    prev_me_field = me_field;
    return me_field;
    
}

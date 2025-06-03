#pragma once

#include <memory>
#include <vector>
#include <string>

struct ScanPoint
{
    double degree;
    double range;
    double intensity;
};

class CLsLidar
{
public:
    CLsLidar();
    ~CLsLidar();

    bool open(const std::string& port);
    bool init();

    bool polling();

    const std::vector<ScanPoint>& get() const;

private:
    struct impl;
    std::unique_ptr<impl> m_impl;
};
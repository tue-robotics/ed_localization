#include <ed_localization/particle_filter.h>

#include <boost/filesystem.hpp>

#include <tue/config/configuration.h>
#include <tue/config/loaders/yaml.h>
#include <tue/filesystem/crawler.h>
#include <tue/filesystem/path.h>

#include <ros/subscriber.h>

#include <iostream>

namespace fs = boost::filesystem;

void usage()
{
    std::cerr << "usage: particle_analyzer YAML_CONFIG PARTICLE_DIRECTORY";
}

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        usage();
        return 1;
    }
    tue::filesystem::Path yaml_config = argv[1];
    tue::filesystem::Path particle_dir = argv[2];

    if (!yaml_config.isRegularFile())
    {
        std::cerr << "yaml config '" << yaml_config.string() << "' is not a file" << std::endl;
        return 1;
    }

    if (!particle_dir.isDirectory())
    {
        std::cerr << "particle directory '" << yaml_config.string() << "' is not a directory" << std::endl;
        return 1;
    }

    tue::Configuration config;
    bool succes = tue::config::loadFromYAMLFile(yaml_config.string(), config);
    if (!succes)
    {
        std::cerr << "Failed to load yaml config '" << yaml_config.string() << "'" << std::endl << config.error() << std::endl;
        return 1;
    }

    ed_localization::ParticleFilter pf;
    pf.configure(config);

    std::vector<fs::path> paths(fs::directory_iterator(fs::path(particle_dir.string())), fs::directory_iterator{});
    std::sort(paths.begin(), paths.end());
    tue::filesystem::Path particle_path;
    for(const fs::path& path: paths)
    {
        std::cout << "path: '" << path.string() << "'" << std::endl;
    }
}

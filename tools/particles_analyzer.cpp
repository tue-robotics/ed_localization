#include <ed_localization/particle_filter.h>

#include <boost/filesystem.hpp>

#include <tue/config/configuration.h>
#include <tue/config/loaders/yaml.h>
#include <tue/config/write.h>
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

    tue::Configuration cluster_data;

    std::vector<fs::path> paths(fs::directory_iterator(fs::path(particle_dir.string())), fs::directory_iterator{});
    std::sort(paths.begin(), paths.end());
    cluster_data.writeArray("files");
    for(const fs::path& path: paths)
    {
        if (path.extension().string() != ".csv")
            continue;
        pf.loadCSV(path.string());
        cluster_data.addArrayItem();
        cluster_data.setValue("name", path.filename().string());
        cluster_data.writeArray("clusters");
        for (const ed_localization::Cluster& c : pf.clusters())
        {
            cluster_data.addArrayItem();
            cluster_data.setValue("weight", c.weight);
            cluster_data.writeGroup("mean");
            cluster_data.setValue("x", c.mean.getOrigin().x);
            cluster_data.setValue("y", c.mean.getOrigin().y);
            cluster_data.setValue("theta", c.mean.rotation());
            cluster_data.endGroup();
            bool first_item = true;
            std::stringstream ss;
            ss << "[";
            for (const double& v : c.cov.m)
            {
                if (!first_item)
                    ss << ", ";
                ss << v;
                first_item = false;
            }
            ss << "]";
            cluster_data.setValue("cov", ss.str());
            cluster_data.endArrayItem();
        }
        cluster_data.endArray();
        cluster_data.endArrayItem();
    }
    cluster_data.endArray();
    tue::filesystem::Path cluster_data_yaml = particle_dir.join("cluster_data.yaml");
    if (!tue::config::toFile(cluster_data_yaml.string(), cluster_data.data(), tue::config::YAML, 4))
    {
        std::cerr << "Failed to write to '" << cluster_data_yaml.string() << "'" << std::endl;
        return 1;
    }
    return 0;
}

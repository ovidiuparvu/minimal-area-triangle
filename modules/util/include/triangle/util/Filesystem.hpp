#ifndef FILESYSTEM_HPP
#define FILESYSTEM_HPP

#include <boost/filesystem.hpp>

#include <string>
#include <vector>

namespace fs = boost::filesystem;


namespace triangle {

    namespace util {

        //! Class containing methods for interacting with the filesystem
        /*!
         *  This class is using the Boost::Filesystem library.
         */
        class Filesystem {

            public:

                //! Check if the given path is a valid folder path
                /*!
                 * \param path  The given path
                 */
                static bool isValidFolderPath(const std::string &path);

                //! Check if the given path is a valid file path
                /*!
                 * \param path  The given path
                 */
                static bool isValidFilePath(const std::string &path);

                //! Check if the given path is a valid file path and has the given extension
                /*!
                 * \param path      The given path
                 * \param extension The given extension
                 */
                static bool isValidFilePath(const std::string &path, const std::string &extension);

                //! Return the given path as an absolute path in native format
                /*!
                 * \param path The given path
                 */
                static std::string nativeFormatFilePath(const std::string &path);

                //! Get the list of files with the given extension in the provided folder
                /*!
                 * \param folderPath    The path to the folder
                 * \param extension     The given extension
                 */
                static std::vector<std::string> getFilesInFolder(const std::string &folderPath,
                                                                 const std::string &extension);

            private:

                //! Check if the given path is a valid file path
                /*!
                 * \param path  The given path
                 */
                static bool isValidFilePath(const fs::path &path);


                // Constants
                static const std::string ERR_INVALID_PATH;

        };

    };

};



#endif

/* Copyright 2014 Gareth Cross

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License. */

#include "tileloader.h"

#include <ros/package.h>
#include <ros/ros.h>

#include <QDir>
#include <QFile>
#include <QImage>
#include <QImageReader>
#include <QNetworkProxy>
#include <QNetworkRequest>
#include <QUrl>
#include <QVariant>
#include <boost/regex.hpp>
#include <functional>  // for std::hash
#include <stdexcept>

static size_t replaceRegex(const boost::regex& ex, std::string& str,
                           const std::string& replace) {
  std::string::const_iterator start = str.begin(), end = str.end();
  boost::match_results<std::string::const_iterator> what;
  boost::match_flag_type flags = boost::match_default;
  size_t count = 0;
  while (boost::regex_search(start, end, what, ex, flags)) {
    str.replace(what.position(), what.length(), replace);
    start = what[0].second;
    count++;
  }
  return count;
}

void TileLoader::MapTile::abortLoading() {
  if (reply_) {
    reply_->abort();
    reply_ = nullptr;
  }
}

bool TileLoader::MapTile::hasImage() const { return !image_.isNull(); }

TileLoader::TileLoader(const std::string& service, double latitude,
                       double longitude, unsigned int zoom, unsigned int blocks,
                       QObject* parent)
    : QObject(parent),
      latitude_(latitude),
      longitude_(longitude),
      zoom_(zoom),
      blocks_(blocks),
      object_uri_(service) {
  assert(blocks_ >= 0);

  const std::string package_path = ros::package::getPath("rviz_satellite");
  if (package_path.empty()) {
    throw std::runtime_error("package 'rviz_satellite' not found");
  }

  std::hash<std::string> hash_fn;
  cache_path_ =
      QDir::cleanPath(QString::fromStdString(package_path) + QDir::separator() +
                      QString("mapscache") + QDir::separator() +
                      QString::number(hash_fn(object_uri_)));

  QDir dir(cache_path_);
  if (!dir.exists() && !dir.mkpath(".")) {
    throw std::runtime_error("Failed to create cache folder: " +
                             cache_path_.toStdString());
  }

  /// @todo: some kind of error checking of the URL

  //  calculate center tile coordinates
  double x, y;
  latLonToTileCoords(latitude_, longitude_, zoom_, x, y);
  center_tile_x_ = std::floor(x);
  center_tile_y_ = std::floor(y);
  //  fractional component
  origin_offset_x_ = x - center_tile_x_;
  origin_offset_y_ = y - center_tile_y_;
}

bool TileLoader::insideCentreTile(double lat, double lon) const {
  double x, y;
  latLonToTileCoords(lat, lon, zoom_, x, y);
  return (std::floor(x) == center_tile_x_ && std::floor(y) == center_tile_y_);
}

void TileLoader::start() {
  //  discard previous set of tiles and all pending requests
  abort();

  ROS_INFO("loading %d blocks around tile=(%d,%d)", blocks_, center_tile_x_,
           center_tile_y_);

  qnam_.reset(new QNetworkAccessManager(this));
  QObject::connect(qnam_.get(), SIGNAL(finished(QNetworkReply*)), this,
                   SLOT(finishedRequest(QNetworkReply*)));
  qnam_->proxyFactory()->setUseSystemConfiguration(true);

  //  determine what range of tiles we can load
  const int min_x = std::max(0, center_tile_x_ - blocks_);
  const int min_y = std::max(0, center_tile_y_ - blocks_);
  const int max_x = std::min(maxTiles(), center_tile_x_ + blocks_);
  const int max_y = std::min(maxTiles(), center_tile_y_ + blocks_);

  //  initiate requests
  for (int y = min_y; y <= max_y; y++) {
    for (int x = min_x; x <= max_x; x++) {
      // Generate filename
      const QString full_path = cachedPathForTile(x, y, zoom_);

      // Check if tile is already in the cache
      QFile tile(full_path);
      if (tile.exists()) {
        QImage image(full_path);
        tiles_.push_back(MapTile(x, y, zoom_, image));
      } else {
        const QUrl uri = uriForTile(x, y);
        //  send request
        QNetworkRequest request = QNetworkRequest(uri);
        auto const userAgent =
            QByteArray("rviz_satellite/" RVIZ_SATELLITE_VERSION
                       " (+https://github.com/gareth-cross/"
                       "rviz_satellite)");
        request.setRawHeader(QByteArray("User-Agent"), userAgent);
        QNetworkReply* rep = qnam_->get(request);
        emit initiatedRequest(request);
        tiles_.push_back(MapTile(x, y, zoom_, rep));
      }
    }
  }

  checkIfLoadingComplete();
}

double TileLoader::resolution() const {
  return zoomToResolution(latitude_, zoom_);
}

/// @see http://wiki.openstreetmap.org/wiki/Slippy_map_tilenames
/// For explanation of these calculations.
void TileLoader::latLonToTileCoords(double lat, double lon, unsigned int zoom,
                                    double& x, double& y) {
  if (zoom > 31) {
    throw std::invalid_argument("Zoom level " + std::to_string(zoom) +
                                " too high");
  } else if (lat < -85.0511 || lat > 85.0511) {
    throw std::invalid_argument("Latitude " + std::to_string(lat) + " invalid");
  } else if (lon < -180 || lon > 180) {
    throw std::invalid_argument("Longitude " + std::to_string(lon) +
                                " invalid");
  }

  const double rho = M_PI / 180;
  const double lat_rad = lat * rho;

  unsigned int n = (1 << zoom);
  x = n * ((lon + 180) / 360.0);
  y = n * (1 - (std::log(std::tan(lat_rad) + 1 / std::cos(lat_rad)) / M_PI)) /
      2;
  ROS_DEBUG_STREAM("Center tile coords: " << x << ", " << y);
}

double TileLoader::zoomToResolution(double lat, unsigned int zoom) {
  const double lat_rad = lat * M_PI / 180;
  return 156543.034 * std::cos(lat_rad) / (1 << zoom);
}

void TileLoader::finishedRequest(QNetworkReply* reply) {
  const QNetworkRequest request = reply->request();

  //  find corresponding tile
  const std::vector<MapTile>::iterator it =
      std::find_if(tiles_.begin(), tiles_.end(),
                   [&](const MapTile& tile) { return tile.reply() == reply; });
  if (it == tiles_.end()) {
    //  removed from list already, ignore this reply
    return;
  }
  MapTile& tile = *it;

  if (reply->error() == QNetworkReply::NoError) {
    //  decode an image
    QImageReader reader(reply);
    if (reader.canRead()) {
      QImage image = reader.read();
      tile.setImage(image);
      image.save(cachedPathForTile(tile.x(), tile.y(), tile.z()), "JPEG");
      emit receivedImage(request);
    } else {
      //  probably not an image
      QString err;
      err = "Unable to decode image at " + request.url().toString();
      emit errorOcurred(err);
    }
  } else {
    const QString err = "Failed loading " + request.url().toString() +
                        " with code " + QString::number(reply->error());
    emit errorOcurred(err);
  }

  checkIfLoadingComplete();
}

bool TileLoader::checkIfLoadingComplete() {
  const bool loaded =
      std::all_of(tiles_.begin(), tiles_.end(),
                  [](const MapTile& tile) { return tile.hasImage(); });
  if (loaded) {
    emit finishedLoading();
  }
  return loaded;
}

QUrl TileLoader::uriForTile(int x, int y) const {
  std::string object = object_uri_;
  //  place {x},{y},{z} with appropriate values
  replaceRegex(boost::regex("\\{x\\}", boost::regex::icase), object,
               std::to_string(x));
  replaceRegex(boost::regex("\\{y\\}", boost::regex::icase), object,
               std::to_string(y));
  replaceRegex(boost::regex("\\{z\\}", boost::regex::icase), object,
               std::to_string(zoom_));

  const QString qstr = QString::fromStdString(object);
  return QUrl(qstr);
}

QString TileLoader::cachedNameForTile(int x, int y, int z) const {
  return "x" + QString::number(x) + "_y" + QString::number(y) + "_z" +
         QString::number(z) + ".jpg";
}

QString TileLoader::cachedPathForTile(int x, int y, int z) const {
  return QDir::cleanPath(cache_path_ + QDir::separator() +
                         cachedNameForTile(x, y, z));
}

int TileLoader::maxTiles() const { return (1 << zoom_) - 1; }

void TileLoader::abort() {
  tiles_.clear();
  //  destroy network access manager
  qnam_.reset();
}

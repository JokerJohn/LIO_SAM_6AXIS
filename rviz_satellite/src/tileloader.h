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

#ifndef TILELOADER_H
#define TILELOADER_H

#include <QObject>
#include <QImage>
#include <QNetworkAccessManager>
#include <QString>
#include <QNetworkReply>
#include <vector>
#include <memory>

class TileLoader : public QObject
{
  Q_OBJECT
public:
  class MapTile
  {
  public:
    MapTile(int x, int y, int z, QNetworkReply* reply = nullptr) : x_(x), y_(y), z_(z), reply_(reply)
    {
    }

    MapTile(int x, int y, int z, QImage& image) : x_(x), y_(y), z_(z), reply_(nullptr), image_(image)
    {
    }

    /// X tile coordinate.
    int x() const
    {
      return x_;
    }

    /// Y tile coordinate.
    int y() const
    {
      return y_;
    }

    /// Z tile zoom value.
    int z() const
    {
      return z_;
    }

    /// Network reply.
    const QNetworkReply* reply() const
    {
      return reply_;
    }

    /// Abort the network request for this tile, if applicable.
    void abortLoading();

    /// Has a tile successfully loaded?
    bool hasImage() const;

    /// Image associated with this tile.
    const QImage& image() const
    {
      return image_;
    }
    void setImage(const QImage& image)
    {
      image_ = image;
    }

  private:
    int x_;
    int y_;
    int z_;
    QNetworkReply* reply_;
    QImage image_;
  };

  explicit TileLoader(const std::string& service, double latitude, double longitude, unsigned int zoom,
                      unsigned int blocks, QObject* parent = nullptr);

  /// Start loading tiles asynchronously.
  void start();

  /// Meters/pixel of the tiles.
  double resolution() const;

  /// X index of central tile.
  int centerTileX() const
  {
    return center_tile_x_;
  }

  /// Y index of central tile.
  int centerTileY() const
  {
    return center_tile_y_;
  }

  /// Fraction of a tile to offset the origin (X).
  double originOffsetX() const
  {
    return origin_offset_x_;
  }

  /// Fraction of a tile to offset the origin (Y).
  double originOffsetY() const
  {
    return origin_offset_y_;
  }

  /// Test if (lat,lon) falls inside centre tile.
  bool insideCentreTile(double lat, double lon) const;

  /// Convert lat/lon to a tile index with mercator projection.
  static void latLonToTileCoords(double lat, double lon, unsigned int zoom, double& x, double& y);

  /// Convert latitude and zoom level to ground resolution.
  static double zoomToResolution(double lat, unsigned int zoom);

  /// Path to tiles on the server.
  const std::string& objectURI() const
  {
    return object_uri_;
  }

  /// Current set of tiles.
  const std::vector<MapTile>& tiles() const
  {
    return tiles_;
  }

  /// Cancel all current requests.
  void abort();

signals:

  void initiatedRequest(QNetworkRequest request);

  void receivedImage(QNetworkRequest request);

  void finishedLoading();

  void errorOcurred(QString description);

public slots:

private slots:

  void finishedRequest(QNetworkReply* reply);

private:
  /// Check if loading is complete. Emit signal if appropriate.
  bool checkIfLoadingComplete();

  /// URI for tile [x,y]
  QUrl uriForTile(int x, int y) const;

  /// Get name for cached tile [x,y,z]
  QString cachedNameForTile(int x, int y, int z) const;

  /// Get file path for cached tile [x,y,z].
  QString cachedPathForTile(int x, int y, int z) const;

  /// Maximum number of tiles for the zoom level
  int maxTiles() const;

  double latitude_;
  double longitude_;
  unsigned int zoom_;
  int blocks_;
  int center_tile_x_;
  int center_tile_y_;
  double origin_offset_x_;
  double origin_offset_y_;

  std::shared_ptr<QNetworkAccessManager> qnam_;
  QString cache_path_;

  std::string object_uri_;

  std::vector<MapTile> tiles_;
};

#endif  // TILELOADER_H

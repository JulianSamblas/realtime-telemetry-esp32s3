% Plot the GNSS route recorded by the device on an OpenStreetMap basemap.
% Input: Trayectoria.mat -> columns [time, lat*1e7, lon*1e7, SIV]
% (logged from the Arduino serial monitor during the road test)

load('Trayectoria.mat')
LAT = Trayectoria(:,2) / 10^7;
LON = Trayectoria(:,3) / 10^7;

name = 'openstreetmap';
url = 'https://a.tile.openstreetmap.org/${z}/${x}/${y}.png';
copyright = char(uint8(169));
attribution = copyright + "OpenStreetMap contributors";
addCustomBasemap(name, url, 'Attribution', attribution)

zoomLevel = 16;
player = geoplayer(LAT(100), LON(100), zoomLevel);
player.Basemap = 'openstreetmap';
plotRoute(player, LAT(:), LON(:));

fig = player.Parent;
exportgraphics(fig, 'Trayectoria.png', 'Resolution', 600)

for i = 1:length(LAT(:))
    plotPosition(player, LAT(i), LON(i))
end

# Maintainer: Thomas Gubler <thomasgubler@gmail.com>
pkgname=python2-mavlink-git
pkgver=20140119
pkgrel=1
pkgdesc="Python implementation of the MAVLink protocol"
arch=(any)
url="http://qgroundcontrol.org/mavlink/pymavlink"
license=('LGPL3')
depends=('python2')
makedepends=('git' 'python2' 'python2-setuptools')
optdepends=()
provides=('python2-mavlink-git')
conflicts=()
options=(!emptydirs)

_gitroot="https://github.com/mavlink/mavlink/"
_gitname="mavlink"
_subfoldername="pymavlink"

build() {
  cd "$srcdir"
  msg "Connecting to GIT server..."

  if [ -d $_gitname ] ; then
    cd $_gitname && git pull origin
    msg "The local files are updated."
  else
    git clone $_gitroot $_gitname
  fi

  msg "GIT checkout done or server timeout"

  cd "$srcdir/$_gitname/$_subfoldername"
  git clean -fdx

  msg "Starting make..."
  python2 setup.py build
}

package() {
  cd "$srcdir/$_gitname/$_subfoldername"
  python2 setup.py install --prefix=/usr --root=$pkgdir/ --optimize=1

  install -Dm644 "$srcdir/$_gitname/$_subfoldername/README.txt" "$pkgdir/usr/share/licenses/$pkgname/README.txt"
}

pkgver() {
	cd "$pkgname"
	printf "r%s.%s" "$(git rev-list --count HEAD)" "$(git rev-parse --short HEAD)"
}

# vim:set ts=2 sw=2 et:

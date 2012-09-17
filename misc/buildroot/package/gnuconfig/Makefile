UPLOAD=ftp://ftp-upload.gnu.org/incoming/ftp/

all:
	@echo "Type 'make upload' to upload to the GNU FTP server."

upload:
	gpg --detach-sign config.guess 
	gpg --detach-sign config.sub
	echo "directory: config" | gpg --clearsign > config.guess.directive.asc
	cp config.guess.directive.asc config.sub.directive.asc
	ftp -a -u $(UPLOAD) config.{guess,sub}{,.sig,.directive.asc}
	rm config.{guess,sub}{.sig,.directive.asc}

check:
	cd testsuite && (sh config-sub.sh; sh config-guess.sh) && rm uname

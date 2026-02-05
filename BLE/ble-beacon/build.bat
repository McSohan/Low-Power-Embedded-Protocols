bash -c "nrfutil pkg generate --hw-version 52 --sd-req=0x00  --application $(find . -name \"zephyr.hex\") --application-version 1 app.zip"
nrfutil device program --firmware app.zip --traits nordicDfu
py ./wgan_v1.py >> train_process.txt

echo "Training completed."

cd ..
git add -A
git commit -m "Shell training auto push."
git push

echo "Prepare to shut down the system."

shutdown -s